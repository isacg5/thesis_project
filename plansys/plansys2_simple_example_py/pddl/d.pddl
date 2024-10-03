(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions :conditional-effects)


(:types
robot
location
person
)


(:predicates
(robot_at ?r - robot ?l - location)
(connected ?lo1 ?lo2 - location)
(person_detected ?p - person ?l - location)
(searched ?r - robot ?l - location)
(person_evaluated ?p - person)
(person_evaluated ?p - person)
(battery_unchecked ?r - robot)
(battery_checked ?r - robot)
(is_free ?r - robot)
(approached ?r - robot ?p - person)
)


(:functions

)


(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?r1 ?r2))
        (at start(robot_at ?r ?r1))
        )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:durative-action approach
    :parameters (?l - location ?p - person ?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(person_detected ?p ?l))
        (over all(robot_at ?r ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(approached ?r ?p))
    )
)


(:durative-action search
    :parameters (?r - robot ?l - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (over all(robot_at ?r ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(searched ?r ?l))
    )
)


(:durative-action evaluate
    :parameters (?l - location ?p - person ?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(person_detected ?p ?l))
        (at start(approached ?r ?p))
        (over all(robot_at ?r ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(person_evaluated ?p))
    )
)

(:durative-action check
    :parameters (?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(battery_unchecked ?r))
        (at start(is_free ?r))
        )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(not(battery_unchecked ?r)))
        (at end(battery_checked ?r))
    )
)
)
