# Cooperative Automation Research Mobility Applications (CARMA) Collaboration Opportunities

The Cooperative Automation Research Mobility Applications (CARMA) software platform is designed to be open source to enable collaboration with industry, academia, infrastructure owner operators (IOOs), and other public agencies on cooperative automation applications. Developed on a Robot Operating System (ROS), a flexible framework for writing software, CARMA has an innovative approach to collaboration for automated vehicle development.

# How to Contribute 

Welcome to the CARMAPlatform contributing guide. We want your help to make the CARMA Project great.
There are a few guidelines that we need contributors to follow so that we can have a chance of keeping on top of things.

## Contribution Tasks

* Reporting Issues
* Making Changes
* Submitting Changes
* Merging Changes

### Reporting Issues

* Make sure you have a [GitHub account](https://github.com/signup/free).
* Ensure the issue was not already reported by searching on GitHub under [Issues](https://github.com/usdot-fhwa-stol/CARMAPlatform/issues)
* If you're unable to find an open issue addressing the problem, open a new one.
* Submit a [CARMAPlatform issue](https://github.com/usdot-fhwa-stol/CARMAPlatform/issues/new).
  * Clearly describing the issue
    * Provide a descriptive summary
    * Explain the expected behavior
    * Explain the actual behavior
    * Provide steps to reproduce the actual behavior

### Making Changes

#### Fork the Repository
**Make a fork of our develop branch.** 

* Firstly you need a local fork of the project, so go ahead and press the “Fork” button in GitHub. 
* This will create a copy of the repository in your own GitHub account and you’ll see a note that it’s been forked underneath the project name:
* With the repository forked, you’re ready to clone it so that you have a local working copy of the code base.
 
#### Clone the Repository
**Then clone a local copy of this forked branch.**
* You can alternatively copy the URL by using the green “Clone or download” button from your repository page that you just forked from the original repository page. 
* Once you click the button, you’ll be able to copy the URL by clicking the binder button next to the URL:

#### Create a New Branch
**The number one rule is to put each piece of work on its own branch.**

#### Make Changes Locally
**Do the work, write good commit messages.**

### Submitting Changes
* A Pull Request is the way to notify the project maintainers that you have some work that they should review and add to the project. 
* You’re requesting that they pull your changes in. Read the article ["Using Pull Requests"](https://help.github.com/articles/using-pull-requests) on GitHub.
* To create one, go to your fork of the project, click on the Pull Requests tab, and click the big green “New Pull Request” button.

### Merging Changes

## Branching Model
In order to make the development process efficient, please comply with the branching model described below. On this model, we mainly use five branches - master, develop, feature, release, and hotfix.

![Branching Workflow](docs/image/git_Workflow.png)

**Main branches**

**master** - This branch contains production-ready code of CARMAPlatform.

**develop** - This branch should be kept stable at all times. This is important because new branches are created off of this branch.

**Supporting branches**

**feature**
* These branches are created from develop; each feature branch is used to implement a single task. After completing the task, the feature branch must be merged into develop.
* To avoid ambiguity, we use a specific naming convention for feature branches − they always begin with feature/ followed by a description based on the functionality implemented by the feature. For example, **feature/sign-up-with-email-and-password.** 
* Thanks to this naming, team members can easily tell what code each branch contains.

**release**
* This branch is created every iteration (i.e. sprint) from develop, and when the team rolls out a release it’s deployed to the staging server for testing. A stable release is merged first into the master branch and then into develop.
* The naming convention for this branch starts with release/ followed by its version. For example, **release/v1.0.1.**

**hotfix**
* This branch is created for handling emergency situations – it allows developers to quickly fix something in production. This branch uses master as the parent branch and merges into both master and develop.
* The name of this branch starts with hotfix/ followed by its version. For example, **hotfix/v0.1.1.**










