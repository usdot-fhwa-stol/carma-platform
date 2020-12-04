# CARMA Collaboration Opportunities

Developed on a Robot Operating System (ROS), a flexible framework for writing software, the CARMA initiative will transform transportation, improving efficiency and safety through automated vehicles (AVs) working together.

To academic transportation researchers, public agencies, original equipment manufacturers (OEMs), and infrastructure owner operators (IOOs), CARMA is the Federal Highway Administration’s (FHWA’s) multimodal initiative using open source software (OSS) for the collaborative research and development (R&D) of cooperative driving automation (CDA) to advance the safety, efficiency, and capacity of the entire transportation system. The FHWA developed technology agnostic software for CARMA to enable flexibility and interoperability across original equipment manufacturers (OEMs) and infrastructure owner operators (IOOs) as well as to enable continuous improvement, productivity and greater quality.
The CARMA Collaborative was established in 2019 to create an active community of users advancing CDA. Through a growing community of CARMA users, prospective users, and other stakeholders, the CARMA Collaborative facilitates the application of CARMA OSS and features to accelerate R&D and encourage industry adoption. 

# How to Contribute 

Welcome to the CARMAPlatform contributing guide. There are a few guidelines that we need contributors to follow so that we can have a chance of keeping on top of things.

## Contribution Tasks

* Reporting Issues
* Making Changes
* Submitting Changes
* Merging Changes
* Register as a collaborator

### Reporting Issues

* Make sure you have a [GitHub account](https://github.com/signup/free).
* Ensure the issue was not already reported by searching CARMA Platform [Issues](https://github.com/usdot-fhwa-stol/carma-platform/issues)
* If you're unable to find an open issue addressing the problem, open a new one.
* Submit a [CARMAPlatform issue](<docs/ISSUE_TEMPLATE.md>).
  * Clearly describing the issue
    * Provide a descriptive summary
    * Explain the expected behavior
    * Explain the actual behavior
    * Provide steps to reproduce the actual behavior

### Making Changes

#### Fork the Repository

* First, you need a local fork of the CARMA repository of interest, so go ahead and press the “Fork” button in GitHub. Read the article ["Fork a repo"](https://help.github.com/articles/fork-a-repo/) on GitHub.
* This will create a copy of the repository in your own GitHub account and you’ll see a note that it’s been forked underneath the project name.
* With the repository forked, you’re ready to clone it so that you have a local working copy of the code base.
 
#### Clone the Repository
**Then clone a local copy of this forked branch.**
* You can alternatively copy the URL by using the green “Clone or download” button from your repository page that you just forked from the original repository page. Read the article ["Cloning a repository"](https://help.github.com/articles/cloning-a-repository/) on GitHub. 
* Once you click the button, you’ll be able to copy the URL by clicking the binder button next to the URL:

#### Create a New Branch
**The number one rule is to put each piece of work on its own branch.**
* Check out the branch you want to make modifications to (normally this is the **`develop branch`**).
* Create a branch specific to the issue being addressed. Read the article ["Creating and deleting branches"](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/) on GitHub. 
* For enhancements, name the branch according to the feature e.g. **`feature/123-auto-activate`.**
* For bug fixes, name the branch according to the fix e.g. **`fix/321-admin-notices`.**
* For code that addresses an existing issue, add the Issue number as a prefix e.g. **`feature/123-auto-activate`** or **`fix/321-admin-notices`.**

#### Make Changes Locally
**Do the work, write good commit messages.**
* Open up the project in your favorite text editor, select the file you want to contribute to, and make your changes.
* If your changes include functionality that requires new testing, write the tests as well.
* Commit your changes using a descriptive commit message that begins with the **`issue number being addressed`**.
* When you have finished making your changes, you'll need to push up your changes to a topic branch in your fork of the repository.  Read the article ["Committing and reviewing changes to your project"](https://help.github.com/desktop/guides/contributing-to-projects/committing-and-reviewing-changes-to-your-project/) on GitHub. 
* With them all committed, push them, this pushes everything in that branch up. 

#### Submit Changes
**Let you tell others about changes you've pushed to a GitHub branch.**
* A Pull Request is the way to notify the project maintainers that you have some work that they should review and add to the project. 
* You’re requesting that they pull your changes in. Read the article ["Using Pull Requests"](https://help.github.com/articles/using-pull-requests) on GitHub.
* Then you can go back to your forked branch and issue a pull request to the repository and choose the correct original branch (should be **`develop`**).
* To create one, go to your fork of the project, click on the Pull Requests tab, and click the big green “New Pull Request” button.

#### Merge Changes
**We want your help to make CARMA Platform Project great.**
* All pull requests will be reviewed by the CARMAPlatform team. 
* During the review of your pull request the team member will either merge it, request changes to it, or close it with an explanation. 
    * Remember: we are unable to add untested code to CARMA, as it will add to the CARMA team's technical debt.
* For major changes the reviewer may require additional support from the team, which could cause some delay. 
* We'll do our best to provide updates and feedback throughout the process. 
* Feel free to open pull requests, and the CARMAPlatform team will communicate through it with any comments.

#### Become a Collaborator
**Please fill out this [form](https://forms.gle/2Rd6ii4BgCLCmRpi7) to be added to the CARMA Collaborative.**

Four Contribution Levels
1.	Informative Group Contributor.
This level is for stakeholders interested at an informational level to keep up-to-date on CARMA activities advancing CDA. This level receives CARMA news, product updates, and first access to public CARMA events.
2.	Prospective User.
This level is for stakeholders who are interested in contributing to CARMA research, development, and testing in the future. This level receives the benefits of Informative Group Contributors, and additionally, this level may request support to prepare to contribute to CARMA and will receive first access to select CARMA group events.
3.	User.
This level is for contributors who have contributed code and/or have a vehicle capable of utilizing CARMA. These contributors will also likely be using [ROS Discourse](https://discourse.ros.org/c/carma) for advancing CDA through open and active collaboration. This level receives the benefits of Prospective Users as well as advanced support and access to technical working groups.
4.	Accelerator.
This level is for contributors who are committed to using CARMA, active contributors to GitHub code, provide questions and feedback using ROS Discourse for advancing CDA, and have an automated vehicle with CARMA installed for research, development, and testing. This level receives the benefits of Users as well as dedicated one-on-one support and the ability to work closely with CARMA product development partners.


## Branching Model
Vincent Driessen’s  ["git flow"](https://nvie.com/posts/a-successful-git-branching-model/) branching model is a git branching and release management workflow that helps developers keep track of features, hotfixes and releases in a software projects. We will follow this model and mainly use five branches - master, develop, feature, release, and hotfix. 

### master
* This branch contains production-ready code of CARMAPlatform.
* This branch should be kept stable at all times

### develop
* When the features/fixes are finished they are merged into develop, where we bring them together for testing, before the final push onto master.

### feature/fix
* These branches are created from develop; **each feature/fix branch is used to implement a single task.** After completing the task, the feature/fix branch must be merged into develop.
* To avoid ambiguity, we use a specific naming convention for feature/fix branches − they always begin with feature/fix followed by the  issue number being addressed and a description based on the functionality implemented by the feature/fix. For example: **`feature/123-sign-up-with-email-and-password`** or **`fix/321-admin-notices`.**
* Thanks to this naming, team members can easily tell what code each branch contains.

### release
* A release branch is created whenever a set of functionality is ready for production release. It is normally created from develop and is used to keep the contents stable for final acceptance testing while other development activities may continue on other branches. 
* Once the release is accepted, it will be merged into the master branch and versioned, then merged into develop.
* The naming convention for this branch starts with release/ followed by its version. For example:  **`release/v1.0.1.`**.

### hotfix
* This branch is created for handling emergency situations – it allows developers to quickly fix something in production. This branch uses master as the parent branch and merges into both master and develop.
* The name of this branch starts with hotfix/ followed by its version. For example: **`hotfix/v0.1.1.`** 

## The End
Hope this guide helps you get started in contributing to the CARMAPlatform project! 
