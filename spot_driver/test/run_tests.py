#!/usr/bin/env python3
import typing
import rosunit

test_cases: typing.List[typing.Tuple[str, str, str]] = [
    ("ros_helpers", "ros_helpers_test", "ros_helpers_test.TestSuiteROSHelpers"),
]

for test_case in test_cases:
    rosunit.unitrun(test_case[0], test_case[1], test_case[2])
