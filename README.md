# Wisconsin Robotics -- 2024-2025 Software System
Codebase for the URC rover at UW-Madison, 2024-2025.

## Contributing
# testing for nico
### Branches

- The `master` Branch is protected and will only accept changes through a peer reviewed merge request. `master` should include fully functionally, hardware compatable code. If hardware or software archeture changes make the branch unviable, the commit will be marked at deprecated with the date on the change.
- The `development` branch is unprotected and to be used for untested changes. The branch will be rebased on any changes to `master`.
- Public branches will be named
`<dev type>/<project|misc>-[issue number]-<brief description>`
	- `feature/` branches are clean, buildable, and only rebased before a pull request or when nescessary
	- `dev/` and `bug/` branches are for general development, and do not need clean refs. These branches may require a forced pull.
- The following commit guidelines are required for "feature" branches and recommended for

### Commits

#### Subject Line

- Try to keep the subject to 50 characters, and strictly no less than 72 characters
- Preface the subject line with the affected subsystem. If the change affects multiple subsystems, consider using multiple commits.
- Capitalize the first letter and do not include a period.
- Use the imperative voice: word your change as a command.
	- For example, a bug fix might read **Drivetrain: Fix the encoder position**

#### Body
- If nessecary, include a description of the motivation and the affected changed. When in doubt, include a body.
    - A bug fix would include an explaination of the bug and justification for the change
    - An API change would include an explaintion with the shortcomings of the previous interface
- Wrap the body at 72 characters

#### Footer
- For tested or ready to test changes, include a sign off with `git commit -s`
- When applicable, include the following tags
    - `Coauthored-by <Developer>`
    - `Reported-by <Developer>`
    - `Tested-by <Developer>`
    - `Reviewed-by <Developer>`
    - `Fixes <Commit Hash>`
    - `Depends-on <Commit Hash>`

### Pull Requests
Code will be merged into either `master` or an upstream development branch with a pull request.

#### Atomic Commits

Commits used for development and commits used for maintaince look very difference. Each **Logical Change** should be seperated into a different commit. As a rule of thumb, no line should be changed twice and no commit should affect two subsystems. 

Rebase the branch you intend to merge onto the target branch. Unclean branches can be changed with `git rebase`. Clean branches might warrent the creation of a new branch. Each commit should be justifiable on its own merits. Each commit should be buildable in isolation.

- Multiple revisions of a single change should be squashed into one commit.
- Changes to an API should be seperated into a commit for the API change and a commit for each subsystem that uses the API.
- Bug fixes and preformance enchancements should be seperated into different commits.
- Use the `Depends-on` footer tag to indicate a commit

#### Opening Pull requests

With your clean branch, open a pull request on github. This can be done through the github issue if applicable. 

## Documentation
Overarching system details will be described through Software Projects files.

There is no particular code style guide that we use, however there are specific requirments that we would like to see while developing, such as giving each function a docstring that describes how the function works, its parameters, as well as the return. Code should be commented often, a good rule of thumb is about every 3 to 5 lines of code there should be a line of comment minimum. Also, it is also helpful to type the arguements and return of the function, for example: def add_2_nums(int: a, int: b) -> int: this function specifies that it takes 2 ints and has a return that is also of type int. Additionally, you should make sure your code is readable and sensible; if there's anything that's confusing or unintuitive, use comments to clarify it for future maintainers.

An example of this would look like this function below.\
def add_2_nums(int: a, int: b) -> int:\
&emsp;&emsp;"""\
&emsp;&emsp;Computes the sum of 2 numbers\
&emsp;&emsp;Args:\
&emsp;&emsp;&emsp;&emsp;a (int): Number that will be added onto\
&emsp;&emsp;&emsp;&emsp;b (int): The number you are adding\
&emsp;&emsp;Returns:\
&emsp;&emsp;&emsp;&emsp;int: the summed number\
&emsp;&emsp;"""\
&emsp;&emsp;return a + b
