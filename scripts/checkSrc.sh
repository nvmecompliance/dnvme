#!/bin/bash

./scripts/checkpatch.pl --file -terse --no-tree *.c
./scripts/checkpatch.pl --file -terse --no-tree *.h
