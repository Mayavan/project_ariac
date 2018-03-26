(define (domain project_ariac)

    (:requirements :strips :typing :fluents)

    (:types Robot Tray Bin PartType)

    (:functions
        (No-of-parts-on-bin ?part - PartType ?bin - Bin)
        (No-of-parts-on-tray ?tray - Tray)
        (No-of-parts-in-order ?part - PartType)
    )

    (:predicates
        (gripperempty ?robot - Robot)
        (holding ?robot - Robot ?part - PartType)
        (robotOverTray ?tray - Tray)
        (robotOverBin ?bin - Bin ?part - PartType)
    )

    (:action move-over-bin
        :parameters(
            ?robot - Robot
            ?part - PartType
            ?tray - Tray
            ?bin - Bin)
        :precondition(and
            (gripperempty ?robot)
			(robotOverTray ?tray)
			(>(No-of-parts-on-bin ?part ?bin)0)
			)
        :effect(and
        	(not(robotOverTray ?tray))
			(robotOverBin ?bin ?part))
    )

    (:action move-over-tray
        :parameters(
            ?robot - Robot
            ?part - PartType
            ?tray - Tray
            ?bin - Bin)
        :precondition(and
            (robotOverBin ?bin ?part)
            (holding ?robot ?part))
        :effect(and
            (robotOverTray ?tray)
            (not(robotOverBin ?bin ?part)))
    )

    (:action pickup
        :parameters(
            ?robot - Robot
            ?part - PartType
            ?bin - Bin
            )
        :precondition(and
            (gripperempty ?robot)
            (robotOverBin ?bin ?part))
        :effect(and
            (decrease (No-of-parts-on-bin ?part ?bin) 1)
            (holding ?robot ?part)
			(not(gripperempty ?robot)))
    )

    (:action putdown
        :parameters(
            ?robot - Robot
			?part - PartType
            ?tray - Tray
		)
        :precondition(and
			(holding ?robot ?part)
			(robotOverTray ?tray))
        :effect(and
            (increase (No-of-parts-on-tray ?tray) 1)
			(decrease (No-of-parts-in-order ?part)1)
			(gripperempty ?robot)
			(not(holding ?robot ?part))
       )
	)
)