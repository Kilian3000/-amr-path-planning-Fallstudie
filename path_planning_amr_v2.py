#!/usr/bin/env python3
"""Pfadplanungs-Prototyp für einen omnidirektionalen AMR im polygonalen Lager.

Das Skript ist bewusst weitgehend eigenständig gehalten. Für die eigentliche
Pfadplanung werden nur Standard-Python-Bibliotheken verwendet; matplotlib wird
nur für die Ausgabe der Abbildungen benötigt. Der Konfigurationsraum wird über
eine approximative Zellzerlegung modelliert: Die kontinuierliche Ebene wird auf
ein reguläres Raster abgebildet, jeder freie Rasterpunkt wird zu einem Knoten
im Graphen, und A* sucht auf diesem 8-Nachbar-Graphen einen möglichst kurzen
Pfad.

Eingaben des Planers sind:
    - eine polygonale bzw. rechteckige Lagergrenze mit Breite und Höhe,
    - Hindernispolygone,
    - ein polygonaler Roboter-Footprint und
    - eine Start- und Zielposition.

Der Roboter wird als omnidirektional mit fester Orientierung angenommen. Eine
Konfiguration q besteht deshalb nur aus (x, y). Falls die Orientierung relevant
wäre, müsste der Konfigurationsraum auf (x, y, theta) erweitert werden.
"""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
import json
import math
import os
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# Ein Punkt wird als (x, y)-Tupel angegeben. Ein Polygon ist eine geordnete Liste solcher Punkte.
Point = Tuple[float, float]
Polygon = List[Point]

# Kleine Toleranz für numerische Vergleiche mit Fließkommazahlen.
EPS = 1e-9


@dataclass(frozen=True)
class Warehouse:
    """Polygonales Lagermodell.

    Die äußere Lagergrenze ist rechteckig, weil viele Lager in einem lokalen
    Koordinatensystem sinnvoll über Breite und Höhe beschrieben werden können.
    Die Hindernisse sind einfache Polygone, deren Eckpunkte geordnet angegeben
    werden.
    """

    width: float
    height: float
    obstacles: List[Polygon]


@dataclass
class PlanResult:
    scenario: str
    robot_name: str
    status: str
    raw_path: Optional[List[Point]]
    smooth_path: Optional[List[Point]]
    raw_length: Optional[float]
    smooth_length: Optional[float]
    raw_nodes: Optional[int]
    smooth_nodes: Optional[int]
    expansions: int


# ---------------------------------------------------------------------------
# Geometrie-Hilfsfunktionen
# ---------------------------------------------------------------------------

def rectangle(x1: float, y1: float, x2: float, y2: float) -> Polygon:
    """Gibt ein achsenparalleles Rechteck als Polygon zurück."""
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]


def centred_rectangle(width: float, height: float) -> Polygon:
    """Gibt einen rechteckigen Footprint um den Roboterreferenzpunkt zurück."""
    hw, hh = width / 2.0, height / 2.0
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def translate(poly: Polygon, dx: float, dy: float) -> Polygon:
    """Verschiebt ein Polygon um dx und dy."""
    return [(x + dx, y + dy) for x, y in poly]


def edges(poly: Polygon) -> Iterable[Tuple[Point, Point]]:
    """Liefert alle Polygonkanten als Punktpaare."""
    for i in range(len(poly)):
        yield poly[i], poly[(i + 1) % len(poly)]


def cross(a: Point, b: Point, c: Point) -> float:
    """Zweidimensionales Kreuzprodukt der Vektoren AB und AC."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a: Point, b: Point, p: Point) -> bool:
    return (
        abs(cross(a, b, p)) <= EPS
        and min(a[0], b[0]) - EPS <= p[0] <= max(a[0], b[0]) + EPS
        and min(a[1], b[1]) - EPS <= p[1] <= max(a[1], b[1]) + EPS
    )


def segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """Gibt True zurück, wenn sich die geschlossenen Strecken AB und CD schneiden oder berühren."""
    c1 = cross(a, b, c)
    c2 = cross(a, b, d)
    c3 = cross(c, d, a)
    c4 = cross(c, d, b)

    if (c1 > EPS and c2 < -EPS or c1 < -EPS and c2 > EPS) and (
        c3 > EPS and c4 < -EPS or c3 < -EPS and c4 > EPS
    ):
        return True

    return (
        on_segment(a, b, c)
        or on_segment(a, b, d)
        or on_segment(c, d, a)
        or on_segment(c, d, b)
    )


def point_in_polygon(point: Point, poly: Polygon) -> bool:
    """Punkt-in-Polygon-Test per Ray Casting. Der Rand zählt als innen."""
    x, y = point
    inside = False
    n = len(poly)

    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        if on_segment(a, b, point):
            return True

        xi, yi = a
        xj, yj = b
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + EPS) + xi
        )
        if intersects:
            inside = not inside

    return inside


def polygons_intersect(poly_a: Polygon, poly_b: Polygon) -> bool:
    """Gibt True zurück, wenn sich zwei einfache Polygone schneiden oder eines das andere enthält."""
    for a1, a2 in edges(poly_a):
        for b1, b2 in edges(poly_b):
            if segments_intersect(a1, a2, b1, b2):
                return True

    if point_in_polygon(poly_a[0], poly_b):
        return True
    if point_in_polygon(poly_b[0], poly_a):
        return True

    return False


def polygon_within_warehouse(poly: Polygon, warehouse: Warehouse) -> bool:
    """Prüft, ob ein Polygon vollständig innerhalb der Lagergrenzen liegt."""
    return all(
        -EPS <= x <= warehouse.width + EPS and -EPS <= y <= warehouse.height + EPS
        for x, y in poly
    )


def config_is_blocked(q: Point, warehouse: Warehouse, robot_polygon: Polygon) -> bool:
    """Prüft, ob der Roboter an der Konfiguration q mit Grenze oder Hindernissen kollidiert."""
    placed_robot = translate(robot_polygon, q[0], q[1])

    if not polygon_within_warehouse(placed_robot, warehouse):
        return True

    return any(polygons_intersect(placed_robot, obstacle) for obstacle in warehouse.obstacles)


# ---------------------------------------------------------------------------
# Rasterbasierter A*-Planer
# ---------------------------------------------------------------------------

def grid_index(point: Point, resolution: float) -> Tuple[int, int]:
    """Wandelt eine metrische Position in einen Rasterindex um."""
    return int(round(point[0] / resolution)), int(round(point[1] / resolution))


def grid_point(index: Tuple[int, int], resolution: float) -> Point:
    """Wandelt einen Rasterindex zurück in metrische Koordinaten."""
    return index[0] * resolution, index[1] * resolution


def path_length(path: Optional[Sequence[Point]]) -> Optional[float]:
    """Berechnet die Länge eines Pfads als Summe euklidischer Teilstrecken."""
    if not path:
        return None
    return sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))


def line_is_free(a: Point, b: Point, warehouse: Warehouse, robot_polygon: Polygon, step: float) -> bool:
    """Prüft eine direkte Verbindung zwischen zwei Punkten durch Abtastung auf Kollisionen."""
    distance = math.dist(a, b)
    samples = max(2, int(math.ceil(distance / step)))

    for k in range(samples + 1):
        t = k / samples
        q = (a[0] * (1 - t) + b[0] * t, a[1] * (1 - t) + b[1] * t)
        if config_is_blocked(q, warehouse, robot_polygon):
            return False

    return True


def smooth_path(
    path: Optional[List[Point]],
    warehouse: Warehouse,
    robot_polygon: Polygon,
    resolution: float,
) -> Optional[List[Point]]:
    """Gierige Sichtlinien-Glättung eines Rasterpfads."""
    if path is None or len(path) <= 2:
        return path

    result = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if line_is_free(path[i], path[j], warehouse, robot_polygon, resolution / 2.0):
                break
            j -= 1
        result.append(path[j])
        i = j
    return result


def plan_path(
    warehouse: Warehouse,
    robot_polygon: Polygon,
    start: Point,
    goal: Point,
    resolution: float = 0.20,
) -> Tuple[Optional[List[Point]], Dict[str, object]]:
    """Plant mit A* einen kollisionsfreien Pfad vom Start zum Ziel.

    Dies ist die zentrale Planungsfunktion. Sie erhält polygonale Eingabedaten
    und gibt den Pfad als Liste von (x, y)-Koordinaten des Roboterreferenzpunkts
    zurück.
    """
    nx = int(round(warehouse.width / resolution)) + 1
    ny = int(round(warehouse.height / resolution)) + 1

    # Start und Ziel werden auf das Raster gerundet, damit A* auf Indizes arbeiten kann.
    start_idx = grid_index(start, resolution)
    goal_idx = grid_index(goal, resolution)
    start_q = grid_point(start_idx, resolution)
    goal_q = grid_point(goal_idx, resolution)

    if config_is_blocked(start_q, warehouse, robot_polygon):
        return None, {"status": "start_blocked", "expansions": 0}
    if config_is_blocked(goal_q, warehouse, robot_polygon):
        return None, {"status": "goal_blocked", "expansions": 0}

    # Acht-Nachbarschaft: vier gerade Bewegungen und vier diagonale Bewegungen.
    moves = [
        (1, 0, resolution),
        (-1, 0, resolution),
        (0, 1, resolution),
        (0, -1, resolution),
        (1, 1, resolution * math.sqrt(2)),
        (1, -1, resolution * math.sqrt(2)),
        (-1, 1, resolution * math.sqrt(2)),
        (-1, -1, resolution * math.sqrt(2)),
    ]

    # Prioritätswarteschlange für A*: (f-Kosten, g-Kosten, Rasterindex).
    open_set: List[Tuple[float, float, Tuple[int, int]]] = []
    heappush(open_set, (math.dist(start_q, goal_q), 0.0, start_idx))
    parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start_idx: None}
    best_g: Dict[Tuple[int, int], float] = {start_idx: 0.0}
    closed = set()
    expansions = 0

    while open_set:
        _, current_g, current = heappop(open_set)
        if current in closed:
            continue
        if current == goal_idx:
            path: List[Point] = []
            node: Optional[Tuple[int, int]] = current
            while node is not None:
                path.append(grid_point(node, resolution))
                node = parent[node]
            path.reverse()
            return path, {
                "status": "ok",
                "length": current_g,
                "nodes": len(path),
                "expansions": expansions,
            }

        closed.add(current)
        expansions += 1
        i, j = current

        for di, dj, cost in moves:
            neighbour = (i + di, j + dj)
            if neighbour[0] < 0 or neighbour[0] >= nx or neighbour[1] < 0 or neighbour[1] >= ny:
                continue
            if neighbour in closed:
                continue

            # Der Nachbarknoten wird nur übernommen, wenn der Roboter dort kollisionsfrei steht.
            q = grid_point(neighbour, resolution)
            if config_is_blocked(q, warehouse, robot_polygon):
                continue

            tentative_g = current_g + cost
            if tentative_g < best_g.get(neighbour, float("inf")):
                best_g[neighbour] = tentative_g
                parent[neighbour] = current
                # Euklidische Distanz als zulässige Heuristik für das 8-Nachbar-Raster.
                heuristic = math.dist(q, goal_q)
                heappush(open_set, (tentative_g + heuristic, tentative_g, neighbour))

    return None, {"status": "no_path", "expansions": expansions}


def occupancy_stats(warehouse: Warehouse, robot_polygon: Polygon, resolution: float) -> Dict[str, float]:
    """Zählt freie und blockierte Rasterpunkte für eine Robotergeometrie."""
    nx = int(round(warehouse.width / resolution)) + 1
    ny = int(round(warehouse.height / resolution)) + 1
    total = nx * ny
    blocked = 0

    for i in range(nx):
        for j in range(ny):
            if config_is_blocked(grid_point((i, j), resolution), warehouse, robot_polygon):
                blocked += 1

    return {
        "total": total,
        "free": total - blocked,
        "blocked": blocked,
        "blocked_share": blocked / total,
    }


# ---------------------------------------------------------------------------
# Demonstrationsdaten und Auswertung
# ---------------------------------------------------------------------------

def demo_warehouse() -> Warehouse:
    """Erzeugt die künstliche Lagerumgebung der Fallstudie."""
    # Rechteckige Hindernisse: Regalblöcke, Sackgasse und enge Passage.
    obstacles = [
        rectangle(3, 2, 8, 3),
        rectangle(3, 5, 8, 6),
        rectangle(11, 2, 16, 3),
        rectangle(11, 5, 16, 6),
        rectangle(5, 8, 6, 11),
        rectangle(9, 8, 10, 11),
        rectangle(5, 10, 10, 11),
        rectangle(12, 8.0, 18, 8.7),
        rectangle(12, 9.8, 18, 10.5),
    ]
    return Warehouse(width=20.0, height=12.0, obstacles=obstacles)


def demo_robots() -> Dict[str, Polygon]:
    """Definiert die zwei Roboter-Footprints der Fallstudie."""
    return {
        "R1 compact 0.8 x 0.8 m": centred_rectangle(0.8, 0.8),
        "R2 carrier 1.2 x 1.2 m": centred_rectangle(1.2, 1.2),
    }


def demo_scenarios() -> Dict[str, Tuple[Point, Point]]:
    """Definiert die Start- und Zielpunkte der drei Testszenarien."""
    return {
        "Basis": ((1.0, 1.0), (19.0, 11.2)),
        "Enge Passage": ((11.0, 9.2), (19.0, 9.2)),
        "Sackgasse": ((7.0, 8.6), (18.0, 1.0)),
    }


def run_evaluation(resolution: float = 0.20) -> Tuple[List[PlanResult], Dict[str, Dict[str, float]]]:
    """Führt alle Testszenarien für alle Roboter aus und sammelt Kennzahlen."""
    warehouse = demo_warehouse()
    robots = demo_robots()
    scenarios = demo_scenarios()

    cspace_stats = {
        name: occupancy_stats(warehouse, robot, resolution) for name, robot in robots.items()
    }
    results: List[PlanResult] = []

    for scenario_name, (start, goal) in scenarios.items():
        for robot_name, robot in robots.items():
            raw_path, info = plan_path(warehouse, robot, start, goal, resolution)
            smoothed = smooth_path(raw_path, warehouse, robot, resolution) if raw_path else None
            results.append(
                PlanResult(
                    scenario=scenario_name,
                    robot_name=robot_name,
                    status=str(info["status"]),
                    raw_path=raw_path,
                    smooth_path=smoothed,
                    raw_length=round(float(info["length"]), 2) if info.get("length") is not None else None,
                    smooth_length=round(path_length(smoothed), 2) if smoothed else None,
                    raw_nodes=int(info["nodes"]) if info.get("nodes") is not None else None,
                    smooth_nodes=len(smoothed) if smoothed else None,
                    expansions=int(info.get("expansions", 0)),
                )
            )

    return results, cspace_stats


# ---------------------------------------------------------------------------
# Plot-Erzeugung
# ---------------------------------------------------------------------------

def plot_outputs(results: List[PlanResult], cspace_stats: Dict[str, Dict[str, float]], out_dir: str, resolution: float) -> None:
    """Erzeugt die Abbildungen für Bericht und GitHub-Repository."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon as MplPolygon
    except ImportError:
        print("matplotlib is not installed; skipping plots.")
        return

    os.makedirs(out_dir, exist_ok=True)
    warehouse = demo_warehouse()
    robots = demo_robots()
    scenarios = demo_scenarios()

    def draw_warehouse(ax):
        ax.set_aspect("equal")
        ax.set_xlim(0, warehouse.width)
        ax.set_ylim(0, warehouse.height)
        ax.set_xlabel("x-Position [m]")
        ax.set_ylabel("y-Position [m]")
        ax.grid(True, linewidth=0.4)
        for obstacle in warehouse.obstacles:
            ax.add_patch(MplPolygon(obstacle, closed=True, facecolor="0.75", edgecolor="black"))

    # Abbildung 1: Lagerumgebung und Roboter-Footprints
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_warehouse(ax)
    ax.set_title("Simulationsumgebung mit polygonalen Hindernissen")
    ax.text(0.8, 11.2, "Wareneingang / Startbereich", fontsize=9)
    ax.text(12.2, 11.2, "enge Passage", fontsize=9)
    ax.text(5.2, 7.4, "Sackgasse", fontsize=9)
    ax.add_patch(MplPolygon(translate(robots["R1 compact 0.8 x 0.8 m"], 1.0, 1.0), closed=True, fill=False, edgecolor="black", linewidth=1.5))
    ax.add_patch(MplPolygon(translate(robots["R2 carrier 1.2 x 1.2 m"], 2.0, 1.0), closed=True, fill=False, edgecolor="black", linestyle="--", linewidth=1.5))
    ax.text(0.6, 0.2, "R1", fontsize=8)
    ax.text(1.6, 0.2, "R2", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig01_environment.png"), dpi=200)
    plt.close(fig)

    # Abbildungen 2 und 3: Belegung des Konfigurationsraums
    for idx, (robot_name, robot) in enumerate(robots.items(), start=2):
        nx = int(round(warehouse.width / resolution)) + 1
        ny = int(round(warehouse.height / resolution)) + 1
        grid = [[0 for _ in range(nx)] for _ in range(ny)]
        for i in range(nx):
            for j in range(ny):
                q = grid_point((i, j), resolution)
                grid[j][i] = 1 if config_is_blocked(q, warehouse, robot) else 0

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.imshow(grid, origin="lower", extent=[0, warehouse.width, 0, warehouse.height], cmap="Greys", alpha=0.75, interpolation="nearest")
        draw_warehouse(ax)
        ax.set_title(f"Diskretisierter Konfigurationsraum für {robot_name}")
        stats = cspace_stats[robot_name]
        ax.text(0.6, 0.6, f"frei: {stats['free']:.0f} | blockiert: {stats['blocked']:.0f} ({stats['blocked_share']*100:.1f} %) ", fontsize=9, bbox={"facecolor": "white", "alpha": 0.8})
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"fig0{idx}_cspace_{idx-1}.png"), dpi=200)
        plt.close(fig)

    # Abbildung 4: geplante und geglättete Pfade
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_warehouse(ax)
    ax.set_title("Geglättete Pfadbeispiele aus dem A*-Prototyp")
    line_styles = ["-", "--", "-.", ":", (0, (5, 1)), (0, (3, 1, 1, 1))]
    for k, result in enumerate(results):
        if result.smooth_path:
            xs = [p[0] for p in result.smooth_path]
            ys = [p[1] for p in result.smooth_path]
            label = f"{result.scenario} - {result.robot_name.split()[0]}"
            ax.plot(xs, ys, linestyle=line_styles[k % len(line_styles)], linewidth=2, marker="o", markersize=3, label=label)
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig04_paths.png"), dpi=200)
    plt.close(fig)

    # Abbildung 5: Vergleich der Pfadlängen
    scenario_names = list(scenarios.keys())
    robot_names = list(robots.keys())
    x = list(range(len(scenario_names)))
    width = 0.35
    fig, ax = plt.subplots(figsize=(9, 5))
    for offset, robot_name in zip([-width / 2, width / 2], robot_names):
        values = []
        for scenario_name in scenario_names:
            matching = [r for r in results if r.scenario == scenario_name and r.robot_name == robot_name]
            values.append(matching[0].smooth_length if matching else 0)
        positions = [i + offset for i in x]
        ax.bar(positions, values, width=width, label=robot_name.split()[0])
        for px, value in zip(positions, values):
            ax.text(px, value + 0.25, f"{value:.2f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(scenario_names)
    ax.set_ylabel("geglättete Pfadlänge [m]")
    ax.set_xlabel("Testszenario")
    ax.set_title("Vergleich der geglätteten Pfadlängen")
    ax.legend()
    ax.grid(axis="y", linewidth=0.4)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "fig05_lengths.png"), dpi=200)
    plt.close(fig)


def main() -> None:
    """Startpunkt des Skripts: Auswertung ausführen, JSON speichern und Plots erzeugen."""
    resolution = 0.20
    results, cspace_stats = run_evaluation(resolution)

    print("C-space statistics")
    for robot_name, stats in cspace_stats.items():
        print(
            f"{robot_name}: total={stats['total']:.0f}, free={stats['free']:.0f}, "
            f"blocked={stats['blocked']:.0f}, blocked_share={stats['blocked_share']*100:.1f}%"
        )

    print("\nPath planning scenarios")
    for result in results:
        print(
            f"{result.scenario:14s} | {result.robot_name:24s} | {result.status:3s} | "
            f"raw={result.raw_length} m | smooth={result.smooth_length} m | "
            f"nodes={result.raw_nodes} | smooth_nodes={result.smooth_nodes} | expansions={result.expansions}"
        )

    serialisable = {
        "resolution": resolution,
        "cspace_stats": cspace_stats,
        "paths": [
            {
                "scenario": r.scenario,
                "robot_name": r.robot_name,
                "status": r.status,
                "raw_length": r.raw_length,
                "smooth_length": r.smooth_length,
                "raw_nodes": r.raw_nodes,
                "smooth_nodes": r.smooth_nodes,
                "expansions": r.expansions,
                "smooth_path": r.smooth_path,
            }
            for r in results
        ],
    }
    with open("path_planning_results.json", "w", encoding="utf-8") as fp:
        json.dump(serialisable, fp, indent=2)

    plot_outputs(results, cspace_stats, "figures_amr", resolution)
    print("\nWrote path_planning_results.json and figures_amr/*.png")


if __name__ == "__main__":
    main()
