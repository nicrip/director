
export DIRECTOR_BUILD=$HOME/source/director/build
export DRAKE_BASE=$(grep "DRAKE_SOURCE_DIR.*PATH" $DIRECTOR_BUILD/CMakeCache.txt | cut -d "=" -f 2)

export LD_LIBRARY_PATH=$DIRECTOR_BUILD/install/lib:$DRAKE_BASE/build/install/lib
export PYTHONPATH=$PYTHONPATH:$DIRECTOR_BUILD/install/lib/python2.7/dist-packages:$DRAKE_BASE/build/install/lib/python2.7/dist-packages
export PATH=$DIRECTOR_BUILD/install/bin:$DRAKE_BASE/build/install/bin:$PATH

alias makedirector='make -C $DIRECTOR_BUILD/src/director-build install'
alias runprocman='$DRAKE_BASE/build/install/bin/bot-procman-sheriff -l iiwa_hardware.pmd'
alias runapp='$DIRECTOR_BUILD/install/bin/directorPython -m director.mainwindowapp --script iiwaManipApp.py --director-config $DRAKE_BASE/drake/examples/kuka_iiwa_arm/director_config.json --bot-config iiwaManip.cfg'
