# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi
# ROS stuff:
#source $ROS_WORKSPACE/devel/setup.sh
source /opt/ros/indigo/setup.bash
export LD_LIBRARY_PATH=/home/wyatt/ros_ws/devel/lib:/home/wyatt/ros_ws/devel/lib/x86_64-linux-gnu:/opt/ros/indigo/lib/x86_64-linux-gnu:/opt/ros/indigo/lib
export PATH=/home/wyatt/ros_ws/devel/bin:/opt/ros/indigo/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games

#ROS_PACKAGE_PATH...did not help
export ROS_PACKAGE_PATH=/home/wyatt/ros_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
#export ROS_PACKAGE_PATH=/home/wyatt/ros_ws:/home/wyatt/ros_ws/src:$ROS_PACKAGE_PATH
export ROS_WORKSPACE=/home/wyatt/ros_ws
export CPATH=/home/wyatt/ros_ws/devel/include:$CPATH
#PKG_CONFIG_PATH...did not help
export PKG_CONFIG_PATH=/home/wyatt/ros_ws/devel/lib/x86_64-linux-gnu/pkgconfig:/opt/ros/indigo/lib/x86_64-linux-gnu/pkgconfig:/home/wyatt/ros_ws/devel/lib/pkgconfig:/opt/ros/indigo/lib/pkgconfig

#export PKG_CONFIG_PATH=/home/wyatt/ros_ws/devel/lib/pkgconfig:/home/wyatt/ros_ws/devel/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
export CMAKE_PREFIX_PATH=/home/wyatt/ros_ws/devel:$CMAKE_PREFIX_PATH

#oops.  Had a typo in the PYTHONPATH line.  Resulted in ROS not finding messages, packages, etc
export PYTHONPATH=/home/wyatt/ros_ws/devel/lib/python2.7/dist-packages:/opt/ros/indigo/lib/python2.7/dist-packages
export ROSLISP_PACKAGE_DIRECTORIES=/home/wyatt/ros_ws/devel/share/common-lisp
#export PS1=\[\033[00;33m\][baxter - http://localhost:11311]\[\033[00m\] \[\e]0;\u@\h:\w\a\]${debian_chroot:+($debian_chroot)}\u@\h:\w\$ 
alias cwru_create_pkg='~/ros_ws/src/cwru_baxter/build_scripts/cwru_create_pkg.py'
alias baxter_master='export ROS_MASTER_URI="http://baxter01:11311"'

