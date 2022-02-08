#!/bin/bash

#  Copyright (C) 2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Code below largely based on template from Stack Overflow:
# https://stackoverflow.com/questions/37257551/defining-subcommands-that-take-arguments-in-bash
# Question asked by user 
# DiogoSaraiva (https://stackoverflow.com/users/4465820/diogosaraiva)
# and answered by user 
# Charles Duffy (https://stackoverflow.com/users/14122/charles-duffy)
# Attribution here is in line with Stack Overflow's Attribution policy cc-by-sa found here:
# https://stackoverflow.blog/2009/06/25/attribution-required/

function parse_args {
    while [[ $# -gt 0 ]]; do
        arg="$1"
        case $arg in
            -r|--repos)
                PATH_TO_REPOS_FILE=$(realpath "$2")
                shift
                shift
                ;;
            -v|--version)
                RELEASE_VERSION="$2"
                shift
                shift
                ;;
            -b|--branch)
                RELEASE_BRANCH="$2"
                shift
                shift
                ;;
            -d|--dir)
                WORK_DIR=$(realpath "$2")
                shift
                shift
                ;;
            --assign)
                PR_ASSIGNEE="$2"
                shift
                shift
                ;;
        esac
    done
}

function release_tool__help {
    cat <<"END_OF_HELP"
|------------------------------------------------------------------------------|
| CARMA Release Automation Tool                                                |
|------------------------------------------------------------------------------|
Usage:
./release_tool.sh <COMMAND> <ARGS>

Supported commands:

clone - Clone all repositories referenced in the repos file into the working 
        directory
update_version_numbers - Update the dockerfiles and checkout.bash files to 
                         reference the new release version number
diff - View the Git diff for all repos under the release
commit_version_numbers - Commit the version number updates after a manual review
create_release_prs - Create PRs (assigned using the --assign option) to merge the 
                     release branches into master
tag_repos - Tag the repositories with the new system release tag (component tags
            must be created manually)
build_images - Build the docker images for each dockerized repository
push_images - Push the resulting images to Dockerhub
create_sync_prs - Create PRs (assigned using the --assign option) to merge
                  master back into develop after release

checkout - Checkout all repos to target branch

Arguments:
    -r|--repos   - The path to the .repos file to load the repositories from
    -v|--version - The version number for the release to be processed
    -b|--branch  - The release branch
    -d|--dir     - The working directory for the release process
    --assign     - The Github username of the assignee for the release PRs

Recommended workflow:
0. Cut release branches and conduct CARMA verification testing until passing

1. Clone all repositories
2. Update the version numbers
3. Manually review all changes for correctness
4. Commit and push the version number updates to the release branch
5. Create PRs to merge the release branches into master for each repository
6. Tag each repository's new master commit with the release version
7. Build all docker images
8. Deploy and test docker images to verify integrity of release
9. Push all docker images to Dockerhub
10. Create and merge sync PRs master -> develop

This tool depends on the Github CLI (https://cli.github.com/) and vcstool
(https://github.com/dirk-thomas/vcstool)
END_OF_HELP
}

function clone_repos {
    echo "Cloning repos from $1"    
    vcs import < $1
}

function checkout_branches {
    vcs custom --git --args checkout $1
}

function assert_set {
    local STRING="$1"
    local ERROR_MSG="$2"

    if [[ -z "$STRING" ]]; then
        echo "$ERROR_MSG"
        exit -1
    fi 
}

function release_tool__checkout {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $RELEASE_BRANCH "Branch must be specified with -b <BRANCH>"

    echo "Checking out repos to $RELEASE_BRANCH"
    cd $WORK_DIR
    checkout_branches $RELEASE_BRANCH
}

function release_tool__clone {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $PATH_TO_REPOS_FILE "Path to .repos file must be specified with -f <PATH_TO_FILE>"

    echo "Step #1: Cloning release branches for all repositories"
    cd $WORK_DIR
    clone_repos $PATH_TO_REPOS_FILE
    checkout_branches $RELEASE_BRANCH
}

function replace_docker_image_versions {
  local FILE="$1"
  local IMAGE_VERSION="$2"
  sed -i "s|usdotfhwastoldev|usdotfhwastol|; s|usdotfhwastolcandidate|usdotfhwastol|; s|:[0-9]*\.[0-9]*\.[0-9]*|:$IMAGE_VERSION|g; s|:CARMA[a-zA-Z]*_[0-9]*\.[0-9]*\.[0-9]*|:$IMAGE_VERSION|g; s|:carma-[a-zA-Z]*-[0-9]*\.[0-9]*\.[0-9]*|:$IMAGE_VERSION|g; s|:develop|:$IMAGE_VERSION|g; s|:vanden-plas|:$IMAGE_VERSION|g" \
      $FILE
}

function update_version_numbers {
    local REPOSITORY=$1
    local TARGET_VERSION_ID=$2
    local WORKSPACE=$3 

    cd $WORKSPACE
    cd $REPOSITORY

    if [[ $REPOSITORY == "carma-config/" ]]; then
        echo "Updating carma-configs..."
        for config in */; do
            if [[ -f $config/docker-compose.yml ]]; then
                echo "Updating $config config"
                sed -i "s|usdotfhwastoldev|usdotfhwastol|; s|usdotfhwastolcandidate|usdotfhwastol|; s|:[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:CARMA[a-zA-Z]*_[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:carma-[a-zA-Z]*-[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:develop|:$TARGET_VERSION_ID|g; s|:vanden-plas|:$TARGET_VERSION_ID|g" \
                    $config/docker-compose.yml
                sed -i "s|usdotfhwastoldev|usdotfhwastol|; s|usdotfhwastolcandidate|usdotfhwastol|; s|:[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:CARMA[a-zA-Z]*_[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:carma-[a-zA-Z]*-[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:develop|:$TARGET_VERSION_ID|g; s|:vanden-plas|:$TARGET_VERSION_ID|g" \
                    $config/docker-compose-background.yml
            fi
        done
        return
    fi

    if [[ $REPOSITORY == "carma-messenger/" ]]; then
        echo "Updating carma-messenger-configs..."
        cd carma-messenger-config
        for config in */; do
            if [[ -f $config/docker-compose.yml ]]; then
                echo "Updating $config config"
                sed -i "s|usdotfhwastoldev|usdotfhwastol|; s|usdotfhwastolcandidate|usdotfhwastol|; s|:[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:CARMA[a-zA-Z]*_[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:carma-[a-zA-Z]*-[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:develop|:$TARGET_VERSION_ID|g; s|:vanden-plas|:$TARGET_VERSION_ID|g" \
                    $config/docker-compose.yml
                sed -i "s|usdotfhwastoldev|usdotfhwastol|; s|usdotfhwastolcandidate|usdotfhwastol|; s|:[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:CARMA[a-zA-Z]*_[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:carma-[a-zA-Z]*-[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:develop|:$TARGET_VERSION_ID|g; s|:vanden-plas|:$TARGET_VERSION_ID|g" \
                    $config/docker-compose-background.yml
            fi
        done
	cd ..
        update_version_numbers "carma-messenger-core" "carma-system-$TARGET_VERSION_ID" .
        cd ..
        update_version_numbers "carma-messenger-ui" "carma-system-$TARGET_VERSION_ID" .
    fi

    echo "Updating version numbers in $REPOSITORY to $TARGET_VERSION_ID"

    if [[ -f Dockerfile ]]; then 
        sed -i "s|usdotfhwastoldev|usdotfhwastol|; s|usdotfhwastolcandidate|usdotfhwastol|; s|:[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:CARMA[a-zA-Z]*_[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:carma-[a-zA-Z]*-[0-9]*\.[0-9]*\.[0-9]*|:$TARGET_VERSION_ID|g; s|:develop|:$TARGET_VERSION_ID|g; s|:vanden-plas|:$TARGET_VERSION_ID|g" \
            Dockerfile
    else
        echo "No Dockerfile found for $REPOSITORY"
    fi

    if [[ -f .circleci/config.yml ]]; then 
	replace_docker_image_versions $(realpath .circleci/config.yml) "$TARGET_VERSION_ID"
    else
        echo "No CircleCI config found for $REPOSITORY"
    fi

    if [[ -f docker/checkout.bash ]]; then
        sed -i "/\$BRANCH/ b; /usdot-fhwa-stol/ s|--branch .*|--branch $TARGET_VERSION_ID|g; /usdot-fhwa-stol/ s|vanden-plas|$TARGET_VERSION_ID|g" \
            docker/checkout.bash
    else
        echo "No checkout.bash found for $REPOSITORY"
    fi
}

function release_tool__update_version_numbers {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $RELEASE_VERSION "Release version must be specified with -v <RELEASE_VERSION>"

    cd $WORK_DIR/carma/src
    pwd
    for repo in */; do
        update_version_numbers "$repo" "carma-system-$RELEASE_VERSION" $WORK_DIR/carma/src
    done

    cd $WORK_DIR
    update_version_numbers "autoware.ai/" "carma-system-$RELEASE_VERSION" $WORK_DIR
}

function release_tool__diff {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"

    cd $WORK_DIR
    vcs custom --git --args diff --color=always | less -r
}

function commit_version_numbers {
    repo=$1

    cd $repo

    branch=$(git symbolic-ref --short HEAD)

    # Ensure PRs aren't opened for non-release branches
    #if [[ -z $(echo $branch | grep "release") ]]; then
    #    echo "Not on a release/* branch, exiting."
    #    exit -1
    #fi

    if [[ $repo == "carma-config/" ]]; then
        echo "Committing carma config changes..."
        for config in */; do
            echo "Add changes for $config..."
            if [[ -f $config/docker-compose.yml ]]; then
                git add $config/docker-compose.yml $config/docker-compose-background.yml
            fi
        done

        git commit -m "Updating version numbers for CARMA release $RELEASE_VERSION"
        git push
        return
    fi

    if [[ -f Dockerfile ]]; then
        git add Dockerfile
    fi

    if [[ -f docker/checkout.bash ]]; then
        git add docker/checkout.bash
    fi
    git commit -m "Updating version numbers for CARMA release $RELEASE_VERSION"
    git push
}

function release_tool__commit_version_numbers {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    echo "Step #3: Commiting and pushing version number updates on all repositories"

    cd $WORK_DIR
    commit_version_numbers $WORK_DIR/autoware.ai

    cd $WORK_DIR/carma/src
    for repo in */; do
        commit_version_numbers $WORK_DIR/carma/src/$repo
    done
}

function release_tool__tag_repos {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $RELEASE_VERSION "Release version must be specified with -v <RELEASE_VERSION>"
    echo "Tagging master branch on all repos"

    cd $WORK_DIR

    vcs checkout --git master

    # Handle autoware's use of carma-master
    cd $WORK_DIR/autoware.ai
    git checkout carma-master
    
    cd $WORK_DIR
    vcs pull
    vcs custom --git --args tag -am "carma-system-$RELEASE_VERSION" carma-system-$RELEASE_VERSION
    vcs custom --git --args push --tags
}

function release_tool__build_images {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $RELEASE_VERSION "Release version must be specified with -v <RELEASE_VERSION>"

    local IMAGE_TAG="carma-system-$RELEASE_VERSION"
    # Build CARMA base
    cd $WORK_DIR/carma/src/carma-base
    echo "Building docker image for carma-base..."
    ./docker/build-image.sh -v $IMAGE_TAG

    # Build Autoware AI
    cd $WORK_DIR/autoware.ai
    echo "Building docker image for autoware..."
    ./docker/build-image.sh -v $IMAGE_TAG


    # Build CARMA Messenger
    cd $WORK_DIR/carma/src/carma-messenger
    echo "Building CARMA Messenger..."
    ./carma-messenger-core/docker/build-image.sh -v $IMAGE_TAG
    ./carma-messenger-ui/docker/build-image.sh -v $IMAGE_TAG
    cd carma-messenger-config
    for messenger_config in */; do
    	cd $WORK_DIR/carma/src/carma-messenger/carma-messenger-config/$messenger_config
        if [[ -f build-image.sh ]]; then
            echo "Building config $messenger_config..."
            ./build-image.sh
        fi
    done
    
    # Build the rest
    cd $WORK_DIR/carma/src
    for repo in */; do
        if [[ $repo == "carma-base/" || $repo == "autoware.ai/" || $repo == "carma-config/" || $repo == "carma-messenger/" ]]; then
            # Skip cause we already built this as dependencies
            continue
        fi

        cd $WORK_DIR/carma/src
        cd $repo
        branch=$(git symbolic-ref --short HEAD)
        if [[ -f "docker/build-image.sh" ]]; then 
            # Ensure images aren't build off of non-master branches
            if [[ -z $(echo $branch | grep "master") ]]; then
                echo "Repo $repo not on master branch, skipping..."
                continue
            fi

            if [[ -z $(git describe --exact-match --tags HEAD | grep carma-system) ]]; then
                echo "Repo $repo not tagged with carma-system version, skipping..."
                continue
            fi

            echo "Building docker image for $repo..."
            ./docker/build-image.sh -v $IMAGE_TAG
        else
            echo "No docker build script found for $repo, skipping..."
        fi
    done

    # Build CARMA Config
    cd $WORK_DIR/carma/src/carma-config
    echo "Building CARMA Configs..."
    for carma_config in */; do
        cd $WORK_DIR/carma/src/carma-config/$carma_config
        if [[ -f build-image.sh ]]; then
            echo "Building config $carma_config..."
            ./build-image.sh
        fi
    done

   echo "==== ALL IMAGES BUILT SUCCESSFULLY ===="
}

function release_tool__push_images {
    parse_args $@
    assert_set $RELEASE_VERSION "Release version must be specified with -v <RELEASE_VERSION>"
    echo "Pushing all images to Dockerhub"
    docker image ls | grep "carma-system-$RELEASE_VERSION" | tr -s ' ' | cut -d ' ' -f1,2 | sed -e 's/ /:/g' | xargs -I {} docker push {}
}

function create_pr_for_repo {
    local repo="$1"
    local source_branch="$2"
    local target_branch="$3"
    local assignee="$4"
 
    # TODO: Check for github CLI installation
    #if ! command -V gh &2>/dev/null; then
    #    echo "Github CLI not installed. PR creation will need to be done manually."
    #    exit -1
    #fi

    read -p "Open PR on $repo for merging branch $source_branch into $target_branch (Assigned to $assignee)? (y/N): " -r -n 1
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
            gh pr create --assignee $assignee --reviewer $assignee --head $source_branch --base $target_branch -t "Merge release to $target_branch for release $RELEASE_VERSION" -b "# PR Details
## Description
Merge PR to formalize release of $source_branch into $target_branch as part of CARMA release process for $RELEASE_VERSION. 

PR created automatically via CARMA release tool and Github CLI
## Motivation and Context

This PR brings the tested and reviewed contents of the $source_branch release/candidate into $target_branch for final release preparation

## How Has This Been Tested?

This release branch (minus final version number changes) has been tested through the CARMA verification test plan.

## Checklist:
- [X] I have added any new packages to the sonar-scanner.properties file
- [X] My change requires a change to the documentation.
- [X] I have updated the documentation accordingly.
- [X] I have read the **CONTRIBUTING** document.
[CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Contributing.md) 
- [X] I have added tests to cover my changes.
- [X] All new and existing tests passed."
            echo "PR created for $repo"
        else
            echo "PR not created for $repo."
        fi
}

function release_tool__create_release_prs {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $RELEASE_VERSION "Release version must be specified with -v <RELEASE_VERSION>"
    assert_set "$PR_ASSIGNEE" "PR assignee must be specified with --assign <ASSIGNEE>"

    echo "Creating Github pull requests for all repositories"

    cd $WORK_DIR/autoware.ai
    branch=$(git symbolic-ref --short HEAD)

    # Ensure PRs aren't opened for non-release branches
    if [[ -z $(echo $branch | grep "release") ]]; then
        echo "$repo not on a release/* branch, exiting."
        exit -1
    fi

    ceate_pr_for_repo autoware.ai $branch carma-master $PR_ASSIGNEE

    cd $WORK_DIR/carma/src
    for repo in */; do
        echo $repo
        cd $WORK_DIR/carma/src
        cd $repo
        branch=$(git symbolic-ref --short HEAD)

        # Ensure PRs aren't opened for non-release branches
        if [[ -z $(echo $branch | grep "release") ]]; then
            echo "$repo not on a release/* branch, exiting."
            exit -1
        fi

	create_pr_for_repo $repo $branch master $PR_ASSIGNEE
    done
}


function release_tool__create_sync_prs {
    parse_args $@
    assert_set "$WORK_DIR" "Work directory must be specified with -d <WORK_DIR>"
    assert_set "$RELEASE_VERSION" "Release version must be specified with -v <RELEASE_VERSION>"
    assert_set "$PR_ASSIGNEE" "PR assignee must be specified with --assign <ASSIGNEE>"

    echo "Creating Github pull requests for all repositories back into develop"
    cd $WORK_DIR/autoware.ai
    branch=$(git symbolic-ref --short HEAD)

    # Ensure PRs aren't opened for non-release branches
    if [[ -z $(echo $branch | grep "master") ]]; then
        echo "$repo not on a master branch, exiting."
        exit -1
    fi

    create_pr_for_repo autoware.ai carma-master carma-develop $PR_ASSIGNEE

    cd $WORK_DIR/carma/src
    for repo in */; do
        echo $repo
        cd $WORK_DIR/carma/src
        cd $repo
        branch=$(git symbolic-ref --short HEAD)

        # Ensure PRs aren't opened for non-release branches
        if [[ -z $(echo $branch | grep "master") ]]; then
            echo "$repo not on a master branch, exiting."
            exit -1
        fi

	create_pr_for_repo $repo master develop $PR_ASSIGNEE
    done
}

release_tool() {
    local cmdname=$1; shift
    if type "release_tool__$cmdname" >/dev/null 2>&1; then
        "release_tool__$cmdname" "$@"
    else
        release_tool__help
        exit -1
    fi
}

# if the functions above are sourced into an interactive interpreter, the user can
# just call "carma-config set" or "carma-config reset" with no further code needed.

# if invoked as a script rather than sourced, call function named on argv via the below;
# note that this must be the first operation other than a function definition
# for $_ to successfully distinguish between sourcing and invocation:
[[ $_ != $0 ]] && return

# Globals
PATH_TO_REPOS_FILE="$(realpath ../carma-platform.repos)"
WORK_DIR=""
RELEASE_VERSION=""
RELEASE_BRANCH=""
PR_ASSIGNEE=""

# make sure we actually *did* get passed a valid function name
if declare -f "release_tool__$1" >/dev/null 2>&1; then
  # invoke that function, passing arguments through
  "release_tool__$@" # same as "$1" "$2" "$3" ... for full argument list
else
    release_tool__help
    exit -1
fi
