(define (domain project_ariac)

    (:requirements :strips :typing :fluents)

    (:types
        robot
        container
        agvs
        part
        order
    )

    (:functions
        (No-of-parts-in-order ?order - order)
        (No-of-part-on-agv ?agv - agvs)
    )

    (:predicates
        (orderContain ?order - order ?part - part)
        (partOnContainer ?part - part ?container - container)
        (setTarget ?container - container ?part - part)
        (isRobotAvailable ?robot - robot)
        (picked ?robot - robot ?part - part)
        (placeed ?part - part ?agv - agvs)
    )

    (:action findPart
        :parameters(
            ?order - order
            ?part - part
            ?container - container)
        :precondition(and
            (partOnContainer ?part ?container)
            ( orderContain ?order ?part))
        :effect(and
            (setTarget ?container ?part)
            (not(partOnContainer ?part ?container)))
    )

    (:action pickup
        :parameters(
            ?robot - robot
            ?part - part
            ?container - container)
        :precondition(and
            (setTarget ?container ?part)
            (isRobotAvailable ?robot))
        :effect(and
            (picked ?robot ?part)
            (not(setTarget ?container ?part))
            (not(isRobotAvailable ?robot)))
    )

    (:action place
        :parameters(
            ?robot - robot
            ?part - part
            ?agv - agvs
            ?order - order)
        :precondition(and
            (picked ?robot ?part))
        :effect(and
            (decrease (No-of-parts-in-order ?order) 1)
            (increase (No-of-part-on-agv ?agv) 1)
            (not( orderContain ?order ?part))
            (not(picked ?robot ?part))
            (isRobotAvailable ?robot))
    )
)
