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

clone - Clone all repositories referenced in the .repos file into the working directory
create_release_branches - Create release branches from the latest develop branches for all cloned repositories
checkout - Checkout all repositories to the specified branch
update_envs
 - Update .env files in carma-cloud, carma-messenger, and carma-config repositories
diff - View Git diff across all repositories under the release
create_release_prs - Create PRs (assigned using the --assign option) to merge the release branches into master
tag_repos - Tag master branches on all repositories with the new system release version
create_sync_prs - Create PRs (assigned using the --assign option) to merge master back into develop after release

Arguments:
    -r|--repos   - The path to the .repos file to load repositories from
    -v|--version - The version number for the release to be processed (e.g., 4.0.0)
    -b|--branch  - The release branch name (e.g., release/tempest)
    -d|--dir     - The working directory for the release process
    --assign     - The GitHub username of the assignee for the release PRs

Recommended workflow:
0. Capture repositories included in the release in a .repos file 
   (comment out or remove repositories not included for the current release)
1. Clone all repositories from the .repos file into a working directory
   Example:
       ./release_tool.sh clone -d /home/user/releases/2025-q4 -r /home/user/releases/carma-platform.repos
2. Create release branches for all cloned repositories (e.g., release/2025-q4)
   Example:
       ./release_tool.sh create_release_branches -d /home/user/releases/2025-q4 -b release/2025-q4
3. Checkout all repositories to the release branch (if not already on it)
   Example:
       ./release_tool.sh checkout -d /home/user/releases/2025-q4 -b release/2025-q4
4. Create and checkout an intermediate branch from the release branch 
   (e.g., update_config_releasename) to update .env files in carma-cloud, carma-messenger, and carma-config
   Example:
       ./release_tool.sh update_envs -d /home/user/releases/2025-q4 -b release/2025-q4 (Candidate)
       ./release_tool.sh update_envs -d /home/user/releases/2025-q4 -b release/2025-q4 -v 5.3.0 (Before Merging master PR's)
5. Commit and push .env file updates to the intermediate branch, then create PRs 
   to merge the intermediate branch into the release branch
   (Handled automatically by the update_envs command)
6. Create PRs to merge release branches into master for each repository
   Example:
       ./release_tool.sh create_release_prs -d /home/user/releases/2025-q4 -v 5.3.0 --assign your-github-username
7. Tag each repository's new master commit with the release version
   Example:
       ./release_tool.sh tag_repos -d /home/user/releases/2025-q4 -v 5.3.0
8. Create Sync branches (e.g., Sync_master_to_dev_2025-q4) from master and update .env files
   in carma-cloud, carma-messenger, and carma-config to point to dev (usdotfhwastoldev)
   Example:
       ./release_tool.sh create_develop_sync_branches -d /home/user/releases/2025-q4 -b release/2025-q4
       ./release_tool.sh update_sync_envs -d /home/user/releases/2025-q4 -b release/2025-q4
9. Merge Sync PRs from intermediate Sync branches (master changes) → develop
   Example:
       ./release_tool.sh create_sync_prs -d /home/user/releases/2025-q4 -b release/2025-q4 --assign your-github-username
This tool depends on:
- GitHub CLI (https://cli.github.com/)
- vcstool (https://github.com/dirk-thomas/vcstool)
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

function release_tool__update_envs {
  parse_args "$@"
  assert_set "$WORK_DIR" "Work directory must be specified with -d <WORK_DIR>"
  assert_set "$RELEASE_BRANCH" "Release branch must be specified with -b <BRANCH>"

  echo "Updating .env files for ${RELEASE_BRANCH} (version: ${RELEASE_VERSION:-<candidate>})"
  update_env_files "carma-cloud/"     "$WORK_DIR/src"
  update_env_files "carma-messenger/" "$WORK_DIR/src"
  update_env_files "carma-config/"    "$WORK_DIR/src"
  echo "Done. Temp branches pushed; open PRs into ${RELEASE_BRANCH}."
}

function release_tool__clone {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $PATH_TO_REPOS_FILE "Path to .repos file must be specified with -r <PATH_TO_FILE>"

    echo "Step #1: Cloning repositories listed in $PATH_TO_REPOS_FILE"
    cd "$WORK_DIR" || exit 1
    clone_repos "$PATH_TO_REPOS_FILE"
    echo "✅ Cloned all repositories into $WORK_DIR/src"
}

function release_tool__create_release_branches {
    parse_args "$@"
    assert_set "$WORK_DIR" "Work directory must be specified with -d <WORK_DIR>"
    assert_set "$RELEASE_BRANCH" "Release branch must be specified with -b <BRANCH>"

    echo "Step #2: Creating release branches for all repositories in $WORK_DIR/src"

    cd "$WORK_DIR/src" || exit 1
    for repo in */ ; do
        echo "Processing $repo ..."
        cd "$repo" || continue
        git fetch origin
        git checkout develop 2>/dev/null || git checkout carma-develop
        git checkout -b "$RELEASE_BRANCH"
        git push origin "$RELEASE_BRANCH"
        cd ..
    done

    echo "✅ Created release branches for all repositories."
}

function release_tool__checkout {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $RELEASE_BRANCH "Branch must be specified with -b <BRANCH>"

    echo "Checking out repos to $RELEASE_BRANCH"
    cd "$WORK_DIR/src" || exit 1
    checkout_branches "$RELEASE_BRANCH"
    vcs custom --git --args pull --ff-only origin "$RELEASE_BRANCH"
    echo "✅ Checked out all repositories to $RELEASE_BRANCH"
}
create_env_update_pr() {
  local repo_path="$1" base_branch="$2" tmp_branch="$3" title="$4" body="$5" repo="$6"
  (
    cd "$repo_path" || exit 0
    gh pr create -R "$(git remote get-url origin | sed -E 's#(git@github\.com:|https?://github\.com/)##; s/\.git$//')" --head "$tmp_branch" --base "$base_branch" --title "$title" --body "$body"
  )
}

function update_env_files {
  local REPOSITORY="$1"
  local WORKSPACE="$2"
  local rel_name="${RELEASE_BRANCH##*/}"
  local ORG TAG SUFFIX=""
  if [[ -n "$RELEASE_VERSION" ]]; then
    ORG="usdotfhwastol"
    TAG="carma-system-${RELEASE_VERSION}"
    SUFFIX="_master"
  else
    ORG="usdotfhwastolcandidate"
    TAG="${rel_name}"
  fi

  cd "$WORKSPACE" || { echo "[ERR] $WORKSPACE"; return; }
  cd "$REPOSITORY" || { echo "[SKIP] $WORKSPACE/$REPOSITORY"; return; }

  git fetch origin
  git checkout "$RELEASE_BRANCH" || { echo "[ERR] missing $RELEASE_BRANCH"; return; }
  git pull --ff-only origin "$RELEASE_BRANCH"
  local TMP="update_config_${rel_name}${SUFFIX}"
  git checkout -B "$TMP"

  if [[ "$REPOSITORY" == "carma-cloud/" ]]; then
    if [[ -f ".env" ]]; then
      sed -i -E "s|^STOL_ORG=.*|STOL_ORG=${ORG}|; s|^STOL_TAG=.*|STOL_TAG=${TAG}|" .env
      git add .env
    fi
    git commit -m "Update .env for ${RELEASE_BRANCH} (ORG=${ORG}, TAG=${TAG})" || true
    git push -u origin "$TMP" || true
    create_env_update_pr "$(pwd)" "$RELEASE_BRANCH" "$TMP" "Update .env for candidate: ${rel_name}" "Automated PR Created by release tool for .env updates (ORG=${ORG}, TAG=${TAG})."
    return
  fi

  if [[ "$REPOSITORY" == "carma-messenger/" ]]; then
    if [[ -d "carma-messenger-config" ]]; then
      for d in carma-messenger-config/*; do
        [[ -f "$d/.env" ]] || continue
        sed -i -E "s|^DOCKER_ORG=.*|DOCKER_ORG=${ORG}|; s|^DOCKER_TAG=.*|DOCKER_TAG=${TAG}|" "$d/.env"
        git add "$d/.env"
      done
    fi
    git commit -m "Update .env for ${RELEASE_BRANCH} (ORG=${ORG}, TAG=${TAG})" || true
    git push -u origin "$TMP" || true
    create_env_update_pr "$(pwd)" "$RELEASE_BRANCH" "$TMP" "Update .env for candidate: ${rel_name}" "Automated PR Created by release tool for .env updates (ORG=${ORG}, TAG=${TAG})."
    return
  fi

  if [[ "$REPOSITORY" == "carma-config/" ]]; then
    for d in */ ; do
      [[ -f "$d/.env" ]] || continue
      sed -i -E "s|^DOCKER_ORG=.*|DOCKER_ORG=${ORG}|; s|^DOCKER_TAG=.*|DOCKER_TAG=${TAG}|" "$d/.env"
      git add "$d/.env"
    done
    git commit -m "Update .env for ${RELEASE_BRANCH} (ORG=${ORG}, TAG=${TAG})" || true
    git push -u origin "$TMP" || true
    create_env_update_pr "$(pwd)" "$RELEASE_BRANCH" "$TMP" "Update .env for candidate: ${rel_name}" "Automated PR Created by release tool for .env updates (ORG=${ORG}, TAG=${TAG})."
    return
  fi
  echo "[SKIP] $REPOSITORY not managed"
}

function release_tool__diff {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"

    cd $WORK_DIR
    vcs custom --git --args diff --color=always | less -r
}

function release_tool__tag_repos {
    parse_args $@
    assert_set $WORK_DIR "Work directory must be specified with -d <WORK_DIR>"
    assert_set $RELEASE_VERSION "Release version must be specified with -v <RELEASE_VERSION>"
    echo "Tagging master branch on all repos"

    local TAG="carma-system-$RELEASE_VERSION"
    cd $WORK_DIR/src

    vcs checkout --git master

    # Handle autoware's use of carma-master
    if [[ -d "$WORK_DIR/src/autoware.ai" ]]; then
      ( cd "$WORK_DIR/src/autoware.ai" && git fetch origin && git checkout carma-master ) || true
    fi
    
    cd $WORK_DIR
    vcs pull
    vcs custom --git --args tag -am "$TAG" $TAG
    vcs custom --git --args push --tags
}

function create_pr_for_repo {
    local repo="$1"
    local source_branch="$2"
    local target_branch="$3"
    local assignee="$4"
 
    if ! command -v gh >/dev/null 2>&1; then
       echo "Github CLI not installed. PR creation will need to be done manually."
       exit -1
    fi
    read -p "Open PR on $repo for merging branch $source_branch into $target_branch (Assigned to $assignee)? (y/N): " -r -n 1
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
            gh pr create -R "$(git remote get-url origin | sed -E 's#(git@github\.com:|https?://github\.com/)##; s/\.git$//')" --assignee $assignee --reviewer $assignee --head $source_branch --base $target_branch -t "Merge ${RELEASE_BRANCH##*/} into $target_branch for release $RELEASE_VERSION" -b "# PR Details 
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

    cd $WORK_DIR/src/autoware.ai
    branch=$(git symbolic-ref --short HEAD)

    # Ensure PRs aren't opened for non-release branches
    if [[ -z $(echo $branch | grep "release") ]]; then
        echo "$repo not on a release/* branch, exiting."
        exit -1
    fi

    create_pr_for_repo autoware.ai $branch carma-master $PR_ASSIGNEE

    cd $WORK_DIR/src
    for repo in */; do
        [[ $repo == "autoware.ai/" ]] && continue
        echo $repo
        cd $WORK_DIR/src/$repo
        branch=$(git symbolic-ref --short HEAD)
        if [[ -z $(echo $branch | grep "release") ]]; then
            echo "$repo not on a release/* branch, exiting."
            exit -1
        fi

	create_pr_for_repo $repo $branch master $PR_ASSIGNEE
    done
}

function release_tool__create_develop_sync_branches {
  parse_args "$@"
  assert_set "$WORK_DIR" "Work directory must be specified with -d <WORK_DIR>"
  assert_set "$RELEASE_BRANCH" "Release branch must be specified with -b <BRANCH>"

  local rel_name="${RELEASE_BRANCH##*/}"
  local SYNC="Sync_master_to_dev_${rel_name}"
  echo "Creating develop sync branches ($SYNC) for all repositories in $WORK_DIR/src"

  cd "$WORK_DIR/src" || exit 1
  for repo in */ ; do
    echo "Processing $repo ..."
    cd "$repo" || continue
    git fetch origin
    git checkout master 2>/dev/null || git checkout carma-master
    git pull --ff-only  
    git checkout -B "$SYNC"
    git push -u origin "$SYNC" || true
    cd ..
  done

  echo "✅ Created $SYNC on all repos."
}

function update_sync_env_files {
  local REPOSITORY="$1"
  local WORKSPACE="$2"
  local rel_name="${RELEASE_BRANCH##*/}"
  local SYNC="Sync_master_to_dev_${rel_name}"

  local ORG="usdotfhwastoldev"
  local TAG="develop"

  cd "$WORKSPACE" || { echo "[ERR] $WORKSPACE"; return; }
  cd "$REPOSITORY" || { echo "[SKIP] $WORKSPACE/$REPOSITORY"; return; }

  git fetch origin
  git checkout "$SYNC" || { echo "[ERR] missing $SYNC"; return; }
  git pull --ff-only origin "$SYNC"

  local TMP="update_envs_to_dev"
  git checkout -B "$TMP"

  if [[ "$REPOSITORY" == "carma-cloud/" ]]; then
    if [[ -f ".env" ]]; then
      sed -i -E "s|^STOL_ORG=.*|STOL_ORG=${ORG}|; s|^STOL_TAG=.*|STOL_TAG=${TAG}|" .env
      git add .env
      git commit -m "Update .env to point ${ORG}:${TAG}" || true
      git push -u origin "$TMP" || true
      create_env_update_pr "$(pwd)" "$SYNC" "$TMP" \
        "Update .env to point ${ORG}:${TAG}" \
        "Automated PR created to update .env for ${rel_name} sync (ORG=${ORG}, TAG=${TAG})."
    fi
    return
  fi

  if [[ "$REPOSITORY" == "carma-messenger/" ]]; then
    if [[ -d "carma-messenger-config" ]]; then
      for d in carma-messenger-config/*; do
        [[ -f "$d/.env" ]] || continue
        sed -i -E "s|^DOCKER_ORG=.*|DOCKER_ORG=${ORG}|; s|^DOCKER_TAG=.*|DOCKER_TAG=${TAG}|" "$d/.env"
        git add "$d/.env"
      done
      git commit -m "Update .env to point ${ORG}:${TAG}" || true
      git push -u origin "$TMP" || true
      create_env_update_pr "$(pwd)" "$SYNC" "$TMP" \
        "Update .env to point ${ORG}:${TAG}" \
        "Automated PR created to update .env for ${rel_name} sync (ORG=${ORG}, TAG=${TAG})."
    fi
    return
  fi

  if [[ "$REPOSITORY" == "carma-config/" ]]; then
    for d in */ ; do
      [[ -f "$d/.env" ]] || continue
      sed -i -E "s|^DOCKER_ORG=.*|DOCKER_ORG=${ORG}|; s|^DOCKER_TAG=.*|DOCKER_TAG=${TAG}|" "$d/.env"
      git add "$d/.env"
    done
    git commit -m "Update .env to point ${ORG}:${TAG}" || true
    git push -u origin "$TMP" || true
    create_env_update_pr "$(pwd)" "$SYNC" "$TMP" \
      "Update .env to point ${ORG}:${TAG}" \
      "Automated PR created to update .env for ${rel_name} sync (ORG=${ORG}, TAG=${TAG})."
    return
  fi

  echo "[SKIP] $REPOSITORY not managed for .env changes"
}

function release_tool__create_sync_prs {
  parse_args "$@"
  assert_set "$WORK_DIR" "Work directory must be specified with -d <WORK_DIR>"
  assert_set "$RELEASE_BRANCH" "Release branch must be specified with -b <BRANCH>"
  assert_set "$PR_ASSIGNEE" "PR assignee must be specified with --assign <ASSIGNEE>"

  local rel_name="${RELEASE_BRANCH##*/}"
  local SYNC="Sync_master_to_dev_${rel_name}"

  echo "Creating sync PRs ($SYNC -> develop) for all repositories"

  if [[ -d "$WORK_DIR/src/autoware.ai" ]]; then
    cd "$WORK_DIR/src/autoware.ai" || exit 1
    if git show-ref --verify --quiet "refs/heads/$SYNC"; then
      git checkout "$SYNC"
      create_pr_for_repo "autoware.ai" "$SYNC" "carma-develop" "$PR_ASSIGNEE"
    else
      echo "[SKIP] autoware.ai: missing $SYNC branch"
    fi
  fi

  cd "$WORK_DIR/src"
  for repo in */ ; do
    [[ "$repo" == "autoware.ai/" ]] && continue
    echo "$repo"
    cd "$repo" || continue
    if git show-ref --verify --quiet "refs/heads/$SYNC"; then
      git checkout "$SYNC"
      create_pr_for_repo "${repo%/}" "$SYNC" "develop" "$PR_ASSIGNEE"
    else
      echo "[SKIP] ${repo%/}: missing $SYNC branch"
    fi
    cd ..
  done
}

function release_tool__update_sync_envs {
  parse_args "$@"
  assert_set "$WORK_DIR" "Work directory must be specified with -d <WORK_DIR>"
  assert_set "$RELEASE_BRANCH" "Release branch must be specified with -b <BRANCH>"

  echo "Retargeting .env to usdotfhwastoldev:develop on $RELEASE_BRANCH sync branches"
  update_sync_env_files "carma-cloud/"     "$WORK_DIR/src"
  update_sync_env_files "carma-messenger/" "$WORK_DIR/src"
  update_sync_env_files "carma-config/"    "$WORK_DIR/src"
  echo "Done. Opened PRs from Sync_master_to_dev_${RELEASE_BRANCH##*/} → develop."
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
