# Daily Logs

AURORA uses GitHub Issues for all daily reporting. This folder only holds the reporting instructions.

## Create a daily log

1. Open the AURORA repository on GitHub.
2. Open **Issues**.
3. Select **New issue**.
4. Choose **Daily Log**.
5. Fill in **Done**, **Problem**, and **ToDo**.
6. Submit the issue.
7. The workflow formats the title and attempts to assign the author.

Use **Done** for completed work, **Problem** for blockers or help needed, and **ToDo** for the next planned work. Add relevant issue or file links when useful, but do not include credentials, secrets, or sensitive personal information.

The workflow runs when a newly opened issue has the `daily-log` label. It changes the title to `YYYY-MM-DD – username – Daily Log` using the `Europe/Stockholm` date and attempts to assign the issue author. If automatic assignment is not allowed, the log remains open and the workflow writes a warning.

> The repository must have a label named `daily-log`. If it does not exist, a repository administrator must create it on GitHub.
