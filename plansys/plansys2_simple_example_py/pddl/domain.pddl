(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions :conditional-effects)


(:types
robot
location
person
state
consciousness
)

(:constants stand sit lay unknown - state
            conscious unconscious confused - consciousness)

(:predicates
(robot_at ?r - robot ?l - location)
(connected ?lo1 ?lo2 - location)
(person_detected ?p - person ?l - location)
(searched ?r - robot ?l - location)
(person_evaluated ?p - person)
(battery_unchecked ?r - robot)
(battery_checked ?r - robot)
(is_free ?r - robot)
(person_state ?p - person ?s - state)
(person_reported ?p - person)
(next_move ?r - robot ?l - location)
(dialog_finished ?p - person)
(person_dialog ?p - person ?c - consciousness)
)


(:functions

)


(:durative-action move
    :parameters (?r - robot ?r1 ?r2 - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(battery_checked ?r))
        (at start(connected ?r1 ?r2))
        (at start(robot_at ?r ?r1))
        (at start(next_move ?r ?r2))
    )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
        (at end(not(next_move ?r ?r2)))
        (at end(battery_unchecked ?r))
        (at end(not(battery_checked ?r)))
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
        (at end(battery_unchecked ?r))
        (at end(not(battery_checked ?r)))
    )
)


(:durative-action evaluate
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
        (at end(person_evaluated ?p))
    )
)

(:durative-action dialog
    :parameters (?l - location ?p - person ?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(person_evaluated ?p))
        (over all(robot_at ?r ?l))
    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(dialog_finished ?p))
    )
)

(:durative-action report
    :parameters (?l - location ?p - person ?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start(is_free ?r))
        (at start(dialog_finished ?p))
        (over all(robot_at ?r ?l))
    )
    :effect(and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(person_reported ?p))
    )
)

(:durative-action check
    :parameters (?r - robot ?from ?to - location)
    :duration ( = ?duration 5)
    :condition (and
        (at start(battery_unchecked ?r))
        (at start(is_free ?r))
        (at start(robot_at ?r ?from))
        (at start(connected ?from ?to))

    )
    :effect (and
        (at start(not(is_free ?r)))
        (at end(is_free ?r))
        (at end(not(battery_unchecked ?r)))
        (at end(battery_checked ?r))
        (at end(next_move ?r ?to))
    )
)




)
