# Copyright (c) 2020 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Graph nav utility functions"""


def id_to_short_code(id):
    """Convert a unique id to a 2 letter short code."""
    tokens = id.split('-')
    if len(tokens) > 2:
        return '%c%c' % (tokens[0][0], tokens[1][0])
    return None


def pretty_print_waypoints(waypoint_id, waypoint_name, short_code_to_count, localization_id, logger):
    short_code = id_to_short_code(waypoint_id)
    if short_code is None or short_code_to_count[short_code] != 1:
        short_code = '  '  # If the short code is not valid/unique, don't show it.

    logger.info("%s Waypoint name: %s id: %s short code: %s" %
            ('->' if localization_id == waypoint_id else '  ',
            waypoint_name, waypoint_id, short_code))


def find_unique_waypoint_id(short_code, graph, name_to_id, logger):
    """Convert either a 2 letter short code or an annotation name into the associated unique id."""
    if len(short_code) != 2:
        # Not a short code, check if it is an annotation name (instead of the waypoint id).
        if short_code in name_to_id:
            # Short code is an waypoint's annotation name. Check if it is paired with a unique waypoint id.
            if name_to_id[short_code] is not None:
                # Has an associated waypoint id!
                return name_to_id[short_code]
            else:
                logger.error("The waypoint name %s is used for multiple different unique waypoints. Please use" + \
                        "the waypoint id." % (short_code))
                return None
        # Also not an waypoint annotation name, so we will operate under the assumption that it is a
        # unique waypoint id.
        return short_code

    ret = short_code
    for waypoint in graph.waypoints:
        if short_code == id_to_short_code(waypoint.id):
            if ret != short_code:
                return short_code  # Multiple waypoints with same short code.
            ret = waypoint.id
    return ret


def update_waypoints_and_edges(graph, localization_id, logger):
    """Update and print waypoint ids and edge ids."""
    name_to_id = dict()
    edges = dict()

    short_code_to_count = {}
    waypoint_to_timestamp = []
    for waypoint in graph.waypoints:
        # Determine the timestamp that this waypoint was created at.
        timestamp = -1.0
        try:
            timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
        except:
            # Must be operating on an older graph nav map, since the creation_time is not
            # available within the waypoint annotations message.
            pass
        waypoint_to_timestamp.append((waypoint.id,
                                        timestamp,
                                        waypoint.annotations.name))

        # Determine how many waypoints have the same short code.
        short_code = id_to_short_code(waypoint.id)
        if short_code not in short_code_to_count:
            short_code_to_count[short_code] = 0
        short_code_to_count[short_code] += 1

        # Add the annotation name/id into the current dictionary.
        waypoint_name = waypoint.annotations.name
        if waypoint_name:
            if waypoint_name in name_to_id:
                # Waypoint name is used for multiple different waypoints, so set the waypoint id
                # in this dictionary to None to avoid confusion between two different waypoints.
                name_to_id[waypoint_name] = None
            else:
                # First time we have seen this waypoint annotation name. Add it into the dictionary
                # with the respective waypoint unique id.
                name_to_id[waypoint_name] = waypoint.id

    # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
    # fallback to sorting by annotation name.
    waypoint_to_timestamp = sorted(waypoint_to_timestamp, key= lambda x:(x[1], x[2]))

    # Print out the waypoints name, id, and short code in a ordered sorted by the timestamp from
    # when the waypoint was created.
    logger.info('%d waypoints:' % len(graph.waypoints))
    for waypoint in waypoint_to_timestamp:
        pretty_print_waypoints(waypoint[0], waypoint[2], short_code_to_count, localization_id, logger)

    for edge in graph.edges:
        if edge.id.to_waypoint in edges:
            if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
                edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
        else:
            edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
        logger.info("(Edge) from waypoint id: ", edge.id.from_waypoint, " and to waypoint id: ",
                edge.id.to_waypoint)

    return name_to_id, edges
