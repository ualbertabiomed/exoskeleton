#!/usr/bin/env bash
#
# © 2018 Konstantin Gredeskoul, All Rights Reserved.
# MIT License
#
# WARNING: This BASH script is completely optional. You don't need it to build this project.
#
# If you choose to run this script to build the project, run:
#
#     $ ./run.sh
#
# It will clean, build and run the tests.
#
#

( [[ -n ${ZSH_EVAL_CONTEXT} && ${ZSH_EVAL_CONTEXT} =~ :file$ ]] || \
  [[ -n $BASH_VERSION && $0 != "$BASH_SOURCE" ]]) && _s_=1 || _s_=0

export _s_
export ProjectRoot=$(pwd)
export BuildDir="${ProjectRoot}/build/run"
export BashLibRoot="${ProjectRoot}/bin/lib-bash"
export LibBashRepo="https://github.com/kigster/lib-bash"

# We are using an awesome BASH library `lib-bash` for prettifying the output, and
# running commands through their LibRun framework.
exoskeleton::lib-bash() {
  [[ ! -d ${BashLibRoot} ]] && curl -fsSL https://git.io/fxZSi | /usr/bin/env bash
  [[ ! -d ${BashLibRoot} ]] && { 
    printf "Unable to git clone lib-bash repo from ${LibBashRepo}"
    exit 1
  }
  
  if [[ -f ${BashLibRoot}/Loader.bash ]]; then
    cd ${BashLibRoot} > /dev/null
    git reset --hard origin/master 2>&1 | cat >/dev/null
    git pull 2>&1 | cat >/dev/null
    [[ -f Loader.bash ]] && source Loader.bash
    cd ${ProjectRoot}
  else
    printf "\nERROR: unable to find lib-bash library from ${LibBashRepo}!\n"
    exit 1
  fi

  run::set-all show-output-off abort-on-error
}

exoskeleton::header() {
  h1::purple "University of Alberta Biomedical Technologies Development Group: Exoskeleton Project"
  local OIFC=${IFC}
  IFS="|" read -r -a gcc_info <<< "$(gcc --version 2>&1 | tr '\n' '|')"
  export IFC=${OIFC}
  h1 "${bldylw}GCC" "${gcc_info[1]}" "${gcc_info[2]}" "${gcc_info[3]}" "${gcc_info[4]}"
  h1 "${bldylw}GIT:    ${bldblu}$(git --version)"
  h1 "${bldylw}CMAKE:  ${bldblu}$(cmake --version | tr '\n' ' ')"
}

exoskeleton::setup() {
  hl::subtle "Creating Build Folder..."
  run "mkdir -p build/run"
}

exoskeleton::clean() {
  hl::subtle "Cleaning output folders..."
  run 'rm -rf bin/d* include/d* lib/*'
}

exoskeleton::build() {
  run "cd build/run"
  run "cmake ../.. "
  run "make -j 12"
  run "make install | egrep -v 'gmock|gtest'"
  run "cd ${ProjectRoot}"
}

exoskeleton::tests() {
  if [[ -f bin/exoskeleton_tests ]]; then
    run::set-next show-output-on
    run "echo && bin/exoskeleton_tests"
  else
    printf "${bldred}Can't find installed executable ${bldylw}bin/exoskeleton_tests.${clr}\n"
    exit 2
  fi
}

exoskeleton::examples() {
  [[ ! -f bin/exo ]] && {
    error "You don't have the compiled binary yet".
    exit 3
  }

  run::set-all show-output-on

  hr
  run "bin/exo 11 7"
  hr
  run "bin/exo 1298798375 94759897"
  hr
  run "bin/exo 78 17"
  hr

}

main() {
  exoskeleton::lib-bash
  exoskeleton::header
  exoskeleton::setup
  exoskeleton::build
  exoskeleton::tests
  exoskeleton::examples
}

(( $_s_ )) || main
