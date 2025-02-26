#!/bin/bash

#################################################
# CARMA Git Repository Auto-Completion Script
#################################################
#
# README
# ------
#
# This script sets up auto-completion for CARMA repositories from
# the usdot-fhwa-stol GitHub organization. After installation,
# you can use simplified commands like:
#
#   git clone carma-msgs
#
# Instead of typing the full URL:
#
#   git clone https://github.com/usdot-fhwa-stol/carma-msgs
#
# FEATURES:
# - Intercepts git clone commands for CARMA repositories
# - Provides tab completion for CARMA repository names
# - Works with all repositories in the usdot-fhwa-stol organization
#
# INSTALLATION:
# 1. Run this script: ./setup-carma-completion.sh
# 2. Restart your terminal or run: source ~/.bashrc
#
# USAGE:
# - Clone repositories with: git clone carma-msgs
# - Use tab completion: git clone carma-<TAB>
#
# CUSTOMIZATION:
# - Add more repositories to the 'repos' array in the script
# - Modify the CARMA_ORG variable if the organization URL changes
#
# TROUBLESHOOTING:
# - If completion doesn't work, ensure bash-completion is installed:
#   sudo apt-get install bash-completion (for Debian/Ubuntu)
#   or equivalent for your distribution
# - Check that the script is being sourced in ~/.bashrc
#
# Create directory for custom completion scripts if it doesn't exist
mkdir -p ~/.bash_completion.d

# Create the completion script for CARMA repositories
cat > ~/.bash_completion.d/carma_git_completion.sh << 'EOL'
#!/bin/bash

# Define the CARMA GitHub organization URL
CARMA_ORG="https://github.com/usdot-fhwa-stol"

# Function to complete git clone with CARMA repo names
_git_clone_carma_complete() {
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    # Only provide completion for repositories starting with "carma"
    if [[ ${cur} == carma* ]]; then
        # You can add more repositories to this list as needed
        local repos=(
            "carma-msgs"
            "carma-platform"
            "carma-cloud"
            "carma-base"
            "carma-vehicle"
            "carma-config"
            "carma-analytics"
            "carma-utils"
            "carma-web-ui"
            "carma-messenger"
        )

        # Filter repositories based on current input
        COMPREPLY=( $(compgen -W "${repos[*]}" -- ${cur}) )
        return 0
    fi
}

# Function to intercept git clone commands
git() {
    if [[ $1 == "clone" && $2 == carma* && $# -eq 2 ]]; then
        # If the command is "git clone carma-something", convert it to the full URL
        local repo=$2
        command git clone "${CARMA_ORG}/${repo}"
    else
        # Otherwise, pass through to the normal git command
        command git "$@"
    fi
}

# Register the completion function for git
complete -F _git_clone_carma_complete git
EOL

# Make the script executable
chmod +x ~/.bash_completion.d/carma_git_completion.sh

# Add the custom completion directory to .bashrc if it's not already there
if ! grep -q "bash_completion.d" ~/.bashrc; then
    echo "# Load custom bash completions" >> ~/.bashrc
    echo "for file in ~/.bash_completion.d/*; do" >> ~/.bashrc
    echo "    [ -f \"\$file\" ] && source \"\$file\"" >> ~/.bashrc
    echo "done" >> ~/.bashrc
fi

# Source the completion script for the current session
source ~/.bash_completion.d/carma_git_completion.sh

echo "CARMA repositories auto-completion has been set up!"
echo "You can now use 'git clone carma-msgs' and tab completion for other CARMA repositories."
echo "Please restart your terminal or run 'source ~/.bashrc' to enable the changes."
