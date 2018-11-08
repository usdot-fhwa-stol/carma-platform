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
* Ensure the issue was not already reported by searching the `develop` branch on GitHub under [Issues](https://github.com/usdot-fhwa-stol/CARMAPlatform/issues)
* If you're unable to find an open issue addressing the problem, open a new one.
* Submit a [CARMAPlatform issue](<docs/ISSUE_TEMPLATE.md>).
  * Clearly describing the issue
    * Provide a descriptive summary
    * Explain the expected behavior
    * Explain the actual behavior
    * Provide steps to reproduce the actual behavior

### Making Changes

#### Fork the Repository
**Make a fork of our develop branch.** 

* Firstly you need a local fork of the project, so go ahead and press the “Fork” button in GitHub. Read the article ["Fork a repo"](https://help.github.com/articles/fork-a-repo/) on GitHub.
* This will create a copy of the repository in your own GitHub account and you’ll see a note that it’s been forked underneath the project name:
* With the repository forked, you’re ready to clone it so that you have a local working copy of the code base.
 
#### Clone the Repository
**Then clone a local copy of this forked branch.**
* You can alternatively copy the URL by using the green “Clone or download” button from your repository page that you just forked from the original repository page. Read the article ["Cloning a repository"](https://help.github.com/articles/cloning-a-repository/) on GitHub. 
* Once you click the button, you’ll be able to copy the URL by clicking the binder button next to the URL:

#### Create a New Branch
**The number one rule is to put each piece of work on its own branch.**
* Create a branch specific to the issue you are working on. Read the article ["Creating and deleting branches"](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/) on GitHub. 
* For enhancements, name the branch according to the feature e.g. **`feature/auto-activate`.**
* For un-reported bug fixes, add a **`fix-`** prefix e.g. **`fix-admin-notices`.**
* For code that addresses an existing Issue, add the Issue number as a prefix e.g. **`feature/123-auto-activate`** or **`feature/321-fix-admin-notices`.**

#### Make Changes Locally
**Do the work, write good commit messages.**
* Open up the project in your favorite text editor, select the file you want to contribute to, and make your changes.
* Commit your changes using a descriptive commit message.
* When you have finished making your changes, you'll need to push up your changes to a topic branch in your fork of the repository.  Read the article ["Committing and reviewing changes to your project"](https://help.github.com/desktop/guides/contributing-to-projects/committing-and-reviewing-changes-to-your-project/) on GitHub. 
* With them all committed, push them, this pushes everything in that branch up. 

### Submitting Changes
* A Pull Request is the way to notify the project maintainers that you have some work that they should review and add to the project. 
* You’re requesting that they pull your changes in. Read the article ["Using Pull Requests"](https://help.github.com/articles/using-pull-requests) on GitHub.
* Then you can go back to your forked branch and issue a pull request to the original repository and choose the correct original branch (should be **`develop`**).
* To create one, go to your fork of the project, click on the Pull Requests tab, and click the big green “New Pull Request” button.

### Merging Changes
**We want your help to make CARMA Project great.**
* All pull requests will be reviewed by the CARMA team. 
* During the review of your pull request the team member will either merge it, request changes to it, or close it with an explanation. 
* For major changes the reviewer may require additional support from the team, which could cause some delay. 
* We'll do our best to provide updates and feedback throughout the process. 
* Feel free to open pull requests, and the CARMA team will communicate through it with any comments.

## Branching Model
In order to make the development process efficient, please comply with the branching model described below. On this model, we mainly use five branches - master, develop, feature, release, and hotfix.

![Branch Workflow](docs/image/Git_Workflow.png)

### master
* This branch contains production-ready code of CARMAPlatform.
* This branch should be kept stable at all times

This is the latest stable version of CARMAPlatform.

### develop
* This branch contains code for the upcoming release.
* This branch should be kept stable at all times. This is important because new branches are created off of this branch.

### feature
* These branches are created from develop; **each feature branch is used to implement a single task.** After completing the task, the feature branch must be merged into develop.
* To avoid ambiguity, we use a specific naming convention for feature branches − they always begin with feature/ followed by a description based on the functionality implemented by the feature. For example, **feature/sign-up-with-email-and-password.**
* Thanks to this naming, team members can easily tell what code each branch contains.

### release
* This branch is created every iteration (i.e. sprint) from develop, and when the team rolls out a release it’s deployed to the staging server for testing. A stable release is merged first into the master branch and then into develop.
* The naming convention for this branch starts with release/ followed by its version. For example, **release/v1.0.1.**

### hotfix
* This branch is created for handling emergency situations – it allows developers to quickly fix something in production. This branch uses master as the parent branch and merges into both master and develop.
* The name of this branch starts with hotfix/ followed by its version. For example, **hotfix/v0.1.1.**

## The End
Hope this guide helps you get started in contributing to the CARMA project! If you still have questions, don't hesitate to send an email over to info@xxxx.orgjoind.in and we'll get back to you. 


