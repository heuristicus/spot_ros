#!/usr/bin/env python3
PKG = "graph_nav_util"
NAME = "graph_nav_util_test"
SUITE = "graph_nav_util_test.TestSuiteGraphNavUtil"

import unittest
import logging
import spot_driver.graph_nav_util as graph_nav_util
from bosdyn.api.graph_nav import map_pb2


class TestGraphNavUtilShortCode(unittest.TestCase):
    def test_id_to_short_code(self):
        self.assertEqual(graph_nav_util.id_to_short_code("AB-CD-EF"), "AC", "AC!=AC")
        self.assertEqual(graph_nav_util.id_to_short_code("AB-CD-EF-1"), "AC", "AC!=AC")


class TestGraphNavUtilFindUniqueWaypointId(unittest.TestCase):
    def setUp(self):
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)
        self.graph = map_pb2.Graph()
        self.name_to_id = {"ABCDE": "Node1"}

    def test_short_code(self):
        # Test normal short code
        self.assertEqual(
            graph_nav_util.find_unique_waypoint_id(
                "AC", self.graph, self.name_to_id, self.logger
            ),
            "AC",
            "AC!=AC, normal short code",
        )
        # Test annotation name that is known
        self.assertEqual(
            graph_nav_util.find_unique_waypoint_id(
                "ABCDE", self.graph, self.name_to_id, self.logger
            ),
            "Node1",
            "ABCDE!=Node1, known annotation name",
        )
        # Test annotation name that is unknown
        self.assertEqual(
            graph_nav_util.find_unique_waypoint_id(
                "ABCDEF", self.graph, self.name_to_id, self.logger
            ),
            "ABCDEF",
            "ABCDEF!=ABCDEF, unknown annotation name",
        )

    def test_short_code_with_graph(self):
        # Test short code that is in graph
        self.graph.waypoints.add(id="AB-CD-EF")
        self.assertEqual(
            graph_nav_util.find_unique_waypoint_id(
                "AC", self.graph, self.name_to_id, self.logger
            ),
            "AB-CD-EF",
            "AC!=AB-CD-EF, short code in graph",
        )
        # Test short code that is not in graph
        self.assertEqual(
            graph_nav_util.find_unique_waypoint_id(
                "AD", self.graph, self.name_to_id, self.logger
            ),
            "AD",
            "AD!=AD, short code not in graph",
        )
        # Test multiple waypoints with the same short code
        self.graph.waypoints.add(id="AB-CD-EF-1")
        self.assertEqual(
            graph_nav_util.find_unique_waypoint_id(
                "AC", self.graph, self.name_to_id, self.logger
            ),
            "AC",
            "AC!=AC, multiple waypoints with same short code",
        )


class TestGraphNavUtilUpdateWaypointsEdges(unittest.TestCase):
    def setUp(self):
        self.logger = logging.Logger("test_graph_nav_util", level=logging.INFO)

    def test_empty_graph(self):
        # Test empty graph
        self.graph = map_pb2.Graph()
        self.localization_id = ""
        graph_nav_util.update_waypoints_and_edges(
            self.graph, self.localization_id, self.logger
        )
        self.assertEqual(
            len(self.graph.waypoints), 0, "Empty graph should have 0 waypoints"
        )
        self.assertEqual(len(self.graph.edges), 0, "Empty graph should have 0 edges")

    def test_one_waypoint(self):
        # Test graph with 1 waypoint
        self.localization_id = ""
        self.graph = map_pb2.Graph()
        new_waypoint = map_pb2.Waypoint()
        new_waypoint.id = "ABCDE"
        new_waypoint.annotations.name = "Node1"
        self.graph.waypoints.add(
            id=new_waypoint.id, annotations=new_waypoint.annotations
        )
        self.name_to_id, self.edges = graph_nav_util.update_waypoints_and_edges(
            self.graph, self.localization_id, self.logger
        )
        self.assertEqual(
            len(self.graph.waypoints), 1, "Graph with 1 waypoint should have 1 waypoint"
        )
        self.assertEqual(
            len(self.graph.edges), 0, "Graph with 1 waypoint should have 0 edges"
        )
        self.assertEqual(len(self.edges), 0, "Edges should have 0 entries")
        self.assertEqual(len(self.name_to_id), 1, "Name to id should have 1 entry")
        self.assertEqual(
            self.name_to_id["Node1"], "ABCDE", "Name to id should have 1 entry"
        )

    def test_two_waypoints_with_edge(self):
        # Test graph with 2 waypoints and an edge between them
        self.localization_id = ""
        self.graph = map_pb2.Graph()
        new_waypoint = map_pb2.Waypoint()
        new_waypoint.id = "ABCDE"
        new_waypoint.annotations.name = "Node1"
        self.graph.waypoints.add(
            id=new_waypoint.id, annotations=new_waypoint.annotations
        )
        new_waypoint.id = "DE"
        new_waypoint.annotations.name = "Node2"
        self.graph.waypoints.add(
            id=new_waypoint.id, annotations=new_waypoint.annotations
        )
        new_edge = map_pb2.Edge.Id(from_waypoint="ABCDE", to_waypoint="DE")

        self.graph.edges.add(id=new_edge)
        self.name_to_id, self.edges = graph_nav_util.update_waypoints_and_edges(
            self.graph, self.localization_id, self.logger
        )
        self.assertEqual(
            len(self.graph.waypoints),
            2,
            "Graph with 2 waypoints should have 2 waypoints",
        )
        self.assertEqual(
            len(self.graph.edges), 1, "Graph with 2 waypoints should have 1 edge"
        )
        self.assertEqual(len(self.edges), 1, "Edges should have 1 entry")
        self.assertEqual(
            self.edges["DE"][0], "ABCDE", "Edges should point to the correct waypoint"
        )
        self.assertEqual(len(self.name_to_id), 2, "Name to id should have 2 entries")
        self.assertEqual(self.name_to_id["Node1"], "ABCDE", "Name to id entry, ABCDE")
        self.assertEqual(self.name_to_id["Node2"], "DE", "Name to id entry, DE")

    def test_two_waypoints_with_edge_and_localization(self):
        # Test graph with 2 waypoints and an edge between them. Mainly affects the pretty print.
        self.localization_id = "ABCDE"
        self.graph = map_pb2.Graph()
        new_waypoint = map_pb2.Waypoint()
        new_waypoint.id = "ABCDE"
        new_waypoint.annotations.name = "Node1"
        self.graph.waypoints.add(
            id=new_waypoint.id, annotations=new_waypoint.annotations
        )
        new_waypoint.id = "DE"
        new_waypoint.annotations.name = "Node2"
        self.graph.waypoints.add(
            id=new_waypoint.id, annotations=new_waypoint.annotations
        )
        new_edge = map_pb2.Edge.Id(from_waypoint="ABCDE", to_waypoint="DE")

        self.graph.edges.add(id=new_edge)
        self.name_to_id, self.edges = graph_nav_util.update_waypoints_and_edges(
            self.graph, self.localization_id, self.logger
        )
        self.assertEqual(
            len(self.graph.waypoints),
            2,
            "Graph with 2 waypoints should have 2 waypoints",
        )
        self.assertEqual(
            len(self.graph.edges), 1, "Graph with 2 waypoints should have 1 edge"
        )
        self.assertEqual(len(self.edges), 1, "Edges should have 1 entry")
        self.assertEqual(
            self.edges["DE"][0], "ABCDE", "Edges should point to the correct waypoint"
        )
        self.assertEqual(len(self.name_to_id), 2, "Name to id should have 2 entries")
        self.assertEqual(self.name_to_id["Node1"], "ABCDE", "Name to id entry, ABCDE")
        self.assertEqual(self.name_to_id["Node2"], "DE", "Name to id entry, DE")


class TestSuiteGraphNavUtil(unittest.TestSuite):
    def __init__(self) -> None:
        super(TestSuiteGraphNavUtil, self).__init__()

        self.loader = unittest.TestLoader()
        self.addTest(self.loader.loadTestsFromTestCase(TestGraphNavUtilShortCode))
        self.addTest(
            self.loader.loadTestsFromTestCase(TestGraphNavUtilFindUniqueWaypointId)
        )
        self.addTest(
            self.loader.loadTestsFromTestCase(TestGraphNavUtilUpdateWaypointsEdges)
        )


if __name__ == "__main__":
    print("Starting tests!")
    import rosunit

    rosunit.unitrun(PKG, NAME, TestGraphNavUtilShortCode)
    rosunit.unitrun(PKG, NAME, TestGraphNavUtilFindUniqueWaypointId)
    rosunit.unitrun(PKG, NAME, TestGraphNavUtilUpdateWaypointsEdges)

    print("Tests complete!")
