(define (problem project_ariac_problem)
  (:domain project_ariac)
  ;;objects
  (:objects
    gear1 gear2 piston1 piston2 - part
    ur10 - robot
    agv - agvs
    bin1 bin2 - container
    order - order
    )

  ;;initial state
  (:init
    (=(No-of-part-on-agv agv) 0)
    (partOnContainer gear1 bin1)
    (partOnContainer gear2 bin1)
    (partOnContainer piston1 bin2)
    (partOnContainer piston2 bin2)
    (isRobotAvailable ur10)
    (=(No-of-parts-in-order order) 5)
    (orderContain order gear1)
    (orderContain order gear2)
    (orderContain order gear3)
    (orderContain order piston1)
    (orderContain order piston2)





    )

  (:goal (and
      (= (No-of-parts-in-order order) 0)
      )
    )
  )
