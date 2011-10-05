CC=gcc
CFLAGS=-g -W -Wall
APP_NAME := test

SOURCES := test.c test_metrics.c test_alloc.c test_rng_sqxdbl.c test_prp.c

INCLUDE :=

all: $(APP_NAME)

$(APP_NAME): $(SOURCES)
	$(CC) $(CFLAGS) $(SOURCES) -o $(APP_NAME) $(LDFLAGS)

clean:
	rm -f *.o
	rm -f $(APP_NAME)

.PHONY: all clean doc

