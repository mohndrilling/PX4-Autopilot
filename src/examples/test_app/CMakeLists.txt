px4_add_module(
	MODULE examples__test_app
	MAIN test_app
	STACK_MAIN 2000
	SRCS
		test_app.c
	DEPENDS
	)