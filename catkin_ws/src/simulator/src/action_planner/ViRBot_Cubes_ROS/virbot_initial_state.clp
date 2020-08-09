
;************************************************
;*						*
;*	Initial state 				*
;*						*
;*                      J.Savage, UNAM          *
;*						*
;*                      1/5/20                  *
;*                                              *
;************************************************




(deffacts Initial-state-objects-rooms-zones-actors


; Objects definitions
	( item (type Objects) (name deposit)(room bedroom)(image table)( attributes no-pick brown)(pose 6.183334 7.000000 0.0))
	( item (type Objects) (name storage)(room livingroom)(image table)( attributes no-pick brown)(pose 3.183334 2.000000 0.0))
	
	;( item (type Objects) (name blockA)(room bedroom)(zone deposit)(image blockA)(attributes pick)(pose 0.3 0.2 0.0))
	;( item (type Objects) (name blockB)(room bedroom)(zone deposit)(image blockB)(attributes pick)(pose 0.3 0.1 0.0))
	;( item (type Objects) (name blockC)(room bedroom)(zone deposit)(image blockC)(attributes pick)(pose 0.1 0.1 0.0))
	;( item (type Objects) (name blockD)(room livingroom)(zone storage)(image blockD)(attributes pick)(pose 0.7 0.8 0.0))
	;( item (type Objects) (name blockE)(room livingroom)(zone storage)(image blockE)(attributes pick)(pose 0.6 0.8 0.0))
	;( item (type Objects) (name blockF)(room livingoom)(zone storage)(image blockF)(attributes pick)(pose 0.8 0.8 0.0))

 	( item (type Objects) (name apple)(room corridor)(zone amazon)(image apple)(attributes pick)(pose 0.2 1.1 0.0))
	( item (type Objects) (name sushi)(room corridor)(zone amazon)(image sushi)(attributes pick)(pose 0.3 1.1 0.0))
	( item (type Objects) (name milk)(room corridor)(zone amazon)(image milk)(attributes pick)(pose 0.4 1.1 0.0))

	( item (type Objects) (name soap)(room corridor)(zone amazon)(image soap)(attributes pick)(pose 0.2 1.0 0.0))
	( item (type Objects) (name perf)(room corridor)(zone amazon)(image perf)(attributes pick)(pose 0.3 1.0 0.0))
	( item (type Objects) (name sham)(room corridor)(zone amazon)(image sham)(attributes pick)(pose 0.4 1.0 0.0))
	


	( item (type Objects) (name freespace)(room any)(zone any)(image none)(attributes none)(pose 0.0 0.0 0.0))

; Rooms definitions
	( Room (name livingroom)(zone storage)(zones dummy1 frontexit frontentrance storage dummy2)(center 0.50 0.80))
	( Room (name kitchen)(zone deposit)(zones dummy1 frontexit frontentrance deposit dummy2)(center 0.45 0.20))
	( Room (name bedroom)(zone deposit)(zones dummy1 frontexit frontentrance deposit dummy2)(center 0.4 0.10))

	( Room (name corridor)(zone amazon)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.04 0.30))
	( Room (name kitchen)(zone fridge)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.5 1.4))
	( Room (name service)(zone table)(zones dummy1 frontexit frontentrance deposit dummy2)(center 1.65 0.36))

; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 1.048340 1.107002 0.0))

; Furniture definitions
	( item (type Furniture) (name cubestable)(zone bedroom)(image table)( attributes no-pick brown)(pose 6.183334 7.000000 0.0))

; Doors definitions
	( item (type Door) (name outsidedoor) (status closed) )

	( Arm (name left))

;stacks definitions
        ;(stack bedroom deposit blockB blockA blockC)
        ;(stack livingroom storage blockE blockD blockF)

		(stack corridor amazon apple sushi milk)
        (stack corridor amazon soap perf sham)

		(real-stack corridor amazon apple sushi milk)
        (real-stack corridor amazon soap perf sham)

        ;(real-stack bedroom deposit blockB blockA blockC)
        ;(real-stack livingroom storage blockE blockD blockF)


	;(goal-stack 1 corridor amazon milk apple sushi)
	;(goal-stack 2 corridor amazon sham perf soap)

	;(goal-stack 1 bedroom deposit blockD blockA blockF)
	;(goal-stack 2 livingroom storage blockB blockE blockC)

	(goal-stack 1 kitchen fridge sushi apple milk)
	(goal-stack 2 service table soap perf sham)

        (plan (name cubes) (number 0)(duration 0))

)



