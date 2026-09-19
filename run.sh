#!/bin/bash

source .venv/bin/activate && python xrdemo.py "$@" 2>&1 | tee output.log
