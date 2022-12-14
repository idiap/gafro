target_sources(${PROJECT_NAME}
	PRIVATE 
)

add_subdirectory(expressions)
add_subdirectory(util)

install(
	FILES 
		Point.hpp
		Point.hxx

		PointPair.hpp
		PointPair.hxx

		Line.hpp
		Line.hxx

		Circle.hpp
		Circle.hxx

		Plane.hpp
		Plane.hxx

		Sphere.hpp
		Sphere.hxx

		Motor.hpp
		Motor.hxx

		MotorGenerator.hpp
		MotorGenerator.hxx
		
		Rotor.hpp
		Rotor.hxx

		RotorGenerator.hpp
		RotorGenerator.hxx

		Translator.hpp
		Translator.hxx

		TranslatorGenerator.hpp
		TranslatorGenerator.hxx

		Multivector.hpp
		Multivector.hxx
		
		Blades.hpp
		
		MultivectorMatrix.hpp

		# Vector.hpp
		# Grade.hpp
		# Meet.hpp
	DESTINATION 
		${GAFRO_PREFIX_INCLUDE}/gafro/algebra
	COMPONENT
		Devel
)