idf_component_register(
    SRCS "main.c" "display.c" "render.c" "sim.c" "imu.c" "button.c"
    INCLUDE_DIRS "."
)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffast-math)
