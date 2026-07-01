#!/usr/bin/env python3
import argparse
import csv
import json
import math
import random
from collections import defaultdict
from pathlib import Path

EARTH_RADIUS = 6378137.0


def read_field(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    required = {"name", "home", "polygon"}
    missing = required - set(data.keys())
    if missing:
        raise ValueError(f"Field config missing keys: {sorted(missing)}")
    if len(data["polygon"]) < 3:
        raise ValueError("Polygon must contain at least 3 points")
    return data


def mean_lat_lon(points):
    lat = sum(p[0] for p in points) / len(points)
    lon = sum(p[1] for p in points) / len(points)
    return lat, lon


def latlon_to_xy(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * EARTH_RADIUS
    return x, y


def xy_to_latlon(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / EARTH_RADIUS)
    lon = lon0 + math.degrees(x / (EARTH_RADIUS * math.cos(math.radians(lat0))))
    return lat, lon


def rotate_point(x, y, angle_deg):
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    return x * ca - y * sa, x * sa + y * ca


def polygon_edges(poly):
    for i in range(len(poly)):
        yield poly[i], poly[(i + 1) % len(poly)]


def horizontal_intersections(poly, y0):
    xs = []
    for (x1, y1), (x2, y2) in polygon_edges(poly):
        if abs(y2 - y1) < 1e-9:
            continue
        if (y1 <= y0 < y2) or (y2 <= y0 < y1):
            t = (y0 - y1) / (y2 - y1)
            x = x1 + t * (x2 - x1)
            xs.append(x)
    xs.sort()
    segments = []
    for i in range(0, len(xs) - 1, 2):
        segments.append((xs[i], xs[i + 1]))
    return segments


def build_lawnmower_route(polygon_latlon, lane_spacing_m, angle_deg=0.0, margin_m=0.0):
    lat0, lon0 = mean_lat_lon(polygon_latlon)
    poly_xy = [latlon_to_xy(lat, lon, lat0, lon0) for lat, lon in polygon_latlon]
    rot_poly = [rotate_point(x, y, -angle_deg) for x, y in poly_xy]
    ys = [p[1] for p in rot_poly]
    min_y, max_y = min(ys), max(ys)
    y = min_y + margin_m
    lines = []
    lane_index = 0
    while y <= max_y - margin_m + 1e-9:
        segments = horizontal_intersections(rot_poly, y)
        for x1, x2 in segments:
            if x2 - x1 <= 1.0:
                continue
            start = (x1 + margin_m, y)
            end = (x2 - margin_m, y)
            if start[0] >= end[0]:
                continue
            if lane_index % 2 == 0:
                lines.extend([start, end])
            else:
                lines.extend([end, start])
            lane_index += 1
        y += lane_spacing_m
    route_latlon = []
    for x, y in lines:
        xr, yr = rotate_point(x, y, angle_deg)
        lat, lon = xy_to_latlon(xr, yr, lat0, lon0)
        route_latlon.append((lat, lon))
    return route_latlon


def estimate_lane_spacing(altitude_m, camera_fov_deg, side_overlap):
    swath = 2.0 * altitude_m * math.tan(math.radians(camera_fov_deg / 2.0))
    usable = swath * (1.0 - side_overlap)
    return max(usable, 5.0)


def mission_items_from_route(route, altitude_m):
    items = []
    do_jump = 1
    if route:
        lat0, lon0 = route[0]
        items.append({
            "type": "SimpleItem",
            "AMSLAltAboveTerrain": None,
            "Altitude": altitude_m,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 22,
            "doJumpId": do_jump,
            "frame": 3,
            "params": [15, 0, 0, None, lat0, lon0, altitude_m],
        })
        do_jump += 1
    for lat, lon in route:
        items.append({
            "type": "SimpleItem",
            "AMSLAltAboveTerrain": None,
            "Altitude": altitude_m,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 16,
            "doJumpId": do_jump,
            "frame": 3,
            "params": [0, 0, 0, None, lat, lon, altitude_m],
        })
        do_jump += 1
    items.append({
        "type": "SimpleItem",
        "AMSLAltAboveTerrain": None,
        "Altitude": altitude_m,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": 20,
        "doJumpId": do_jump,
        "frame": 2,
        "params": [0, 0, 0, 0, 0, 0, 0],
    })
    return items


def export_qgc_plan(field, route, altitude_m, cruise_speed, out_path: Path):
    home = field["home"]
    mission = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "version": 1,
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {
            "version": 2,
            "firmwareType": field.get("firmwareType", 12),
            "vehicleType": field.get("vehicleType", 2),
            "globalPlanAltitudeMode": 1,
            "cruiseSpeed": cruise_speed,
            "hoverSpeed": field.get("hoverSpeed", 5),
            "plannedHomePosition": [home[0], home[1], home[2]],
            "items": mission_items_from_route(route, altitude_m),
        },
    }
    out_path.write_text(json.dumps(mission, ensure_ascii=False, indent=2), encoding="utf-8")


def create_demo_observations(field, out_csv: Path, n=12):
    polygon = field["polygon"]
    lat_values = [p[0] for p in polygon]
    lon_values = [p[1] for p in polygon]
    min_lat, max_lat = min(lat_values), max(lat_values)
    min_lon, max_lon = min(lon_values), max(lon_values)
    random.seed(42)
    rows = []
    for i in range(1, n + 1):
        rows.append({
            "zone_id": f"Z{i:02d}",
            "timestamp": f"2026-06-{(i % 28) + 1:02d}T09:00:00",
            "lat": round(random.uniform(min_lat, max_lat), 7),
            "lon": round(random.uniform(min_lon, max_lon), 7),
            "ndvi": round(random.uniform(0.24, 0.82), 3),
            "soil_moisture": round(random.uniform(18, 72), 1),
            "canopy_temp": round(random.uniform(21, 37), 1),
            "pest_risk": round(random.uniform(0.05, 0.92), 2),
            "trend": round(random.uniform(-0.22, 0.18), 3),
        })
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def load_observations(path: Path):
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = []
        for row in reader:
            rows.append({
                "zone_id": row["zone_id"],
                "timestamp": row["timestamp"],
                "lat": float(row["lat"]),
                "lon": float(row["lon"]),
                "ndvi": float(row["ndvi"]),
                "soil_moisture": float(row["soil_moisture"]),
                "canopy_temp": float(row["canopy_temp"]),
                "pest_risk": float(row["pest_risk"]),
                "trend": float(row["trend"]),
            })
        return rows


def classify_zone(avg):
    actions = []
    priority = "normal"
    if avg["ndvi"] < 0.35 or avg["trend"] < -0.10:
        actions.append("Проверить состояние растений и провести очное дообследование участка")
        priority = "high"
    if avg["soil_moisture"] < 30 and avg["canopy_temp"] > 31:
        actions.append("Рассмотреть локальную корректировку полива: признаки водного стресса")
        priority = "critical"
    elif avg["soil_moisture"] < 35:
        actions.append("Оценить необходимость дополнительного увлажнения или пересмотра графика полива")
        priority = "high" if priority == "normal" else priority
    if avg["pest_risk"] > 0.70:
        actions.append("Запланировать фитосанитарный осмотр и контроль очагов поражения")
        priority = "critical"
    if 0.35 <= avg["ndvi"] < 0.50:
        actions.append("Проверить обеспеченность элементами питания и неоднородность развития посевов")
        priority = "high" if priority == "normal" else priority
    if avg["trend"] > 0.05 and avg["ndvi"] >= 0.55 and avg["soil_moisture"] >= 35:
        actions.append("Состояние стабильное: продолжать мониторинг в плановом режиме")
    if not actions:
        actions.append("Существенных отклонений не выявлено, рекомендуется штатное наблюдение")
    return priority, " | ".join(actions)


def generate_recommendations(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[row["zone_id"]].append(row)
    result = []
    for zone_id, items in sorted(groups.items()):
        avg = {
            "ndvi": sum(x["ndvi"] for x in items) / len(items),
            "soil_moisture": sum(x["soil_moisture"] for x in items) / len(items),
            "canopy_temp": sum(x["canopy_temp"] for x in items) / len(items),
            "pest_risk": sum(x["pest_risk"] for x in items) / len(items),
            "trend": sum(x["trend"] for x in items) / len(items),
        }
        priority, recommendation = classify_zone(avg)
        result.append({
            "zone_id": zone_id,
            "avg_ndvi": round(avg["ndvi"], 3),
            "avg_soil_moisture": round(avg["soil_moisture"], 1),
            "avg_canopy_temp": round(avg["canopy_temp"], 1),
            "avg_pest_risk": round(avg["pest_risk"], 2),
            "avg_trend": round(avg["trend"], 3),
            "priority": priority,
            "recommendation": recommendation,
        })
    return result


def save_csv(rows, out_path: Path):
    if not rows:
        return
    with out_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_route_csv(route, out_path: Path):
    rows = []
    for idx, (lat, lon) in enumerate(route, start=1):
        rows.append({"wp": idx, "lat": round(lat, 7), "lon": round(lon, 7)})
    save_csv(rows, out_path)


def save_summary(field, route, altitude_m, lane_spacing_m, recs, out_path: Path):
    summary = {
        "project": "Автоматизация мониторинга сельхозугодий с помощью БАС",
        "field_name": field["name"],
        "route_waypoints": len(route),
        "survey_altitude_m": altitude_m,
        "lane_spacing_m": round(lane_spacing_m, 2),
        "critical_zones": sum(1 for r in recs if r["priority"] == "critical"),
        "high_priority_zones": sum(1 for r in recs if r["priority"] == "high"),
        "normal_zones": sum(1 for r in recs if r["priority"] == "normal"),
    }
    out_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")


def parse_args():
    p = argparse.ArgumentParser(
        description="Построение полетного задания БАС и агрорекомендаций по данным мониторинга"
    )
    p.add_argument("--field", default="field_example.json", help="JSON с описанием поля")
    p.add_argument("--observations", default="observations_example.csv", help="CSV с наблюдениями")
    p.add_argument("--output-dir", default="./results", help="Каталог результатов")
    p.add_argument("--altitude", type=float, default=60.0, help="Высота облета, м")
    p.add_argument("--camera-fov", type=float, default=78.0, help="Угол обзора камеры, град")
    p.add_argument("--side-overlap", type=float, default=0.30, help="Боковое перекрытие 0..0.95")
    p.add_argument("--route-angle", type=float, default=0.0, help="Угол маршрутных галсов, град")
    p.add_argument("--margin", type=float, default=4.0, help="Отступ от кромки поля, м")
    p.add_argument("--cruise-speed", type=float, default=12.0, help="Скорость миссии, м/с")
    p.add_argument("--generate-demo-observations", action="store_true", help="Пересоздать демо-наблюдения")
    return p.parse_args()


def main():
    args = parse_args()
    field_path = Path(args.field)
    obs_path = Path(args.observations)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    field = read_field(field_path)

    if args.generate_demo_observations or not obs_path.exists():
        create_demo_observations(field, obs_path)

    lane_spacing_m = estimate_lane_spacing(args.altitude, args.camera_fov, args.side_overlap)
    route = build_lawnmower_route(field["polygon"], lane_spacing_m, args.route_angle, args.margin)
    if len(route) < 2:
        raise RuntimeError("Не удалось построить маршрут. Проверьте контур поля, угол и параметры съемки.")

    export_qgc_plan(field, route, args.altitude, args.cruise_speed, out_dir / "mission.plan")
    save_route_csv(route, out_dir / "route_waypoints.csv")

    observations = load_observations(obs_path)
    recommendations = generate_recommendations(observations)
    save_csv(recommendations, out_dir / "agro_recommendations.csv")
    save_summary(field, route, args.altitude, lane_spacing_m, recommendations, out_dir / "summary.json")

    print("Готово")
    print(f"Поле: {field['name']}")
    print(f"Путевых точек: {len(route)}")
    print(f"Шаг галсов: {lane_spacing_m:.2f} м")
    print(f"Результаты сохранены в: {out_dir.resolve()}")


if __name__ == "__main__":
    main()
