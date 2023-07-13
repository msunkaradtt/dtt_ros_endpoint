#!/bin/bash
printf 'Making model of %s ... \n' "${PWD##*/}"
xacro model.xacro -o model.sdf
printf '... done\n'

