# Upload this project to GitHub

## 1. Create an empty GitHub repository

Suggested repository name:

`ros2-autonomous-lawn-mower`

Do not initialize it with a README, `.gitignore`, or license because those are already included here.

## 2. Open a terminal in this folder

```bash
cd ~/path/to/ros2-autonomous-lawn-mower
```

## 3. Initialize Git

```bash
git init
git add .
git commit -m "Initial release: ROS 2 autonomous lawn mower"
git branch -M main
```

## 4. Add your GitHub repository

SSH:

```bash
git remote add origin git@github.com:BesherTaye/ros2-autonomous-lawn-mower.git
git push -u origin main
```

or HTTPS:

```bash
git remote add origin https://github.com/BesherTaye/ros2-autonomous-lawn-mower.git
git push -u origin main
```

## 5. Choose a license

The ROS package metadata intentionally keeps the license as `TODO` so you can choose the license you want before publishing.
Common open-source choices include Apache-2.0, BSD-3-Clause, and MIT.
