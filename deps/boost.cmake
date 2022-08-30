set(Boost_USE_STATIC_LIBS on)
find_package(
		Boost 1.65 REQUIRED
		COMPONENTS system thread regex filesystem
)
