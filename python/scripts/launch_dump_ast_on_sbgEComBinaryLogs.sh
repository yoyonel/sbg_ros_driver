#!/bin/bash
mkdir -p gen/mako gen/msg
python dump_ast.py ../sbgECom/src/binaryLogs/sbgEComBinaryLogs.h
