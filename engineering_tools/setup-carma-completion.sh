#!/bin/bash

#################################################
# CARMA & STOL Git Repository Auto-Completion Script
#################################################
#
# README
# ------
#
# This script sets up auto-completion for all repositories from
# the usdot-fhwa-stol GitHub organization. After installation,
# you can use simplified commands like:
#
#   git clone carma-msgs
#   git clone autoware.ai
#
# Instead of typing the full URL:
#
#   git clone https://github.com/usdot-fhwa-stol/carma-msgs
#
# FEATURES:
# - Intercepts git clone commands for any STOL repository
# - Provides tab completion for all repository names
# - Works with all repositories in the usdot-fhwa-stol organization
#
# INSTALLATION:
# 1. Run this script: ./setup-stol-completion.sh
# 2. Restart your terminal or run: source ~/.bashrc
#
# USAGE:
# - Clone repositories with: git clone <repo-name>
# - Use tab completion: git clone <TAB>
#
# CUSTOMIZATION:
# - Add more repositories to the 'repos' array in the script
# - Modify the STOL_ORG variable if the organization URL changes
#
# TROUBLESHOOTING:
# - If completion doesn't work, ensure bash-completion is installed:
#   sudo apt-get install bash-completion (for Debian/Ubuntu)
#   or equivalent for your distribution
# - Check that the script is being sourced in ~/.bashrc
#
# Create directory for custom completion scripts if it doesn't exist
mkdir -p ~/.bash_completion.d

# Create the completion script for STOL repositories
cat > ~/.bash_completion.d/stol_git_completion.sh << 'EOL'
#!/bin/bash

# Define the STOL GitHub organization URL
STOL_ORG="https://github.com/usdot-fhwa-stol"

# Define the list of repositories only once
# All repositories in the usdot-fhwa-stol organization
stol_repos=(
    "carma-msgs"
    "carma-utils"
    "carma-platform"
    "carma-cloud"
    "carma-base"
    "carma-vehicle"
    "carma-config"
    "carma-analytics-fotda"
    "carma-web-ui"
    "carma-messenger"
    "autoware.ai"
    "autoware.auto"
    "carma-novatel-oem7-driver-wrapper"
    "carma-velodyne-lidar-driver"
    "cdasim"
    "carma-builds"
    "carma-time-lib"
    "actions"
    "v2x-ros-driver"
    "carma-ssc-interface-wrapper"
    "carma-ns3-adapter"
    "cdasim-config"
    "carma-dbw-mkz-ros"
    "evc-sumo"
    "v2x-ros-conversion"
    "carma-messenger-bridge"
    "stol-j2735"
    "opendrive2lanelet"
    "carma-lightbar-driver"
    "multiple_object_tracking"
    "carma-streets"
    "ns-3_c-v2x"
    "carma-developer-tools"
    "carma-torc-pinpoint-driver"
    "carma-1-tenth"
    "carma-carla-integration"
    "tracetools_analysis"
    "scenario-runner"
    "carla-sensor-lib"
)

# Function to complete git clone with repo names
_git_clone_stol_complete() {
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    # Filter repositories based on current input
    COMPREPLY=( $(compgen -W "${stol_repos[*]}" -- ${cur}) )
    return 0
}

# Function to intercept git clone commands for any repository in the list
git() {
    if [[ $1 == "clone" && $# -eq 2 ]]; then
        # Check if the second argument matches any of our repos
        local repo_match=false
        local repo_name=$2

        for repo in "${stol_repos[@]}"; do
            if [[ "$repo_name" == "$repo" ]]; then
                repo_match=true
                break
            fi
        done

        if [[ "$repo_match" == true ]]; then
            # If the command is cloning one of our repos, convert it to the full URL
            command git clone "${STOL_ORG}/${repo_name}"
        else
            # Otherwise, pass through to the normal git command
            command git "$@"
        fi
    else
        # For non-clone commands, pass through to the normal git command
        command git "$@"
    fi
}

# Register the completion function for git
complete -F _git_clone_stol_complete git
EOL

# Make the script executable
chmod +x ~/.bash_completion.d/stol_git_completion.sh

# Add the custom completion directory to .bashrc if it's not already there
if ! grep -q "bash_completion.d" ~/.bashrc; then
    echo "# Load custom bash completions" >> ~/.bashrc
    echo "for file in ~/.bash_completion.d/*; do" >> ~/.bashrc
    echo "    [ -f \"\$file\" ] && source \"\$file\"" >> ~/.bashrc
    echo "done" >> ~/.bashrc
fi

# Source the completion script for the current session
source ~/.bash_completion.d/stol_git_completion.sh
source ~/.bashrc

echo "STOL repositories auto-completion has been set up!"
echo "You can now use 'git clone <repo-name-in-usdot-fhwa-stol>' and tab completion for all STOL repositories."
