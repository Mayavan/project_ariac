(define (problem project_ariac_problem)
  (:domain project_ariac)
  ;;objects
  (:objects
    gear piston - PartType
    ur10 - Robot
    tray1 tray2  - Tray
    bin1 bin2 - Bin
    )

  ;;initial state
  (:init
    (=(No-of-parts-on-bin piston bin1) 12)
    (=(No-of-parts-on-bin gear bin2) 16)
    (=(No-of-parts-on-tray tray1) 0)
    (gripperempty ur10)
    (robotOverBin bin2 gear)
    (=(No-of-parts-in-order gear) 1)
    (=(No-of-parts-in-order piston) 1)
    )

  (:goal (and
      (= (No-of-parts-in-order piston) 0)
      (= (No-of-parts-in-order gear) 0)
      )
  )
)
