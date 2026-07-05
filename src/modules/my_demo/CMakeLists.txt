############################################################################

############################################################################


px4_add_module(
	MODULE modules__my_demo
	MAIN my_demo
	COMPILE_FLAGS
	SRCS
		MyDemo.cpp
		MyDemo.hpp
	DEPENDS
		controllib
		geo
		SlewRate
	)
