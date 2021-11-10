set(srcs
	"bl.c"
	"io.c"
	"main.c"
	"misc.c"
)

idf_component_register(
	SRCS "${srcs}"
	INCLUDE_DIRS "."
)