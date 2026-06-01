# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import Any

from .state_file import StateFile


def resolve_container_cli() -> str | None:
    """Resolve the container engine executable (Docker or Podman).

    Resolution order (risk-averse defaults):

    1. ``ISAACLAB_CONTAINER_CLI`` if set to an absolute executable path, or a name found on ``PATH``.
    2. ``docker`` on ``PATH`` (preserves existing behavior for all standard installs).
    3. ``podman`` on ``PATH`` (drop-in for sites that ship Podman without a ``docker`` shim).

    Returns:
        Absolute path to the container CLI, or None if nothing usable was found.
    """
    env_cli = os.environ.get("ISAACLAB_CONTAINER_CLI", "").strip()
    if env_cli:
        if os.path.isabs(env_cli) and os.path.isfile(env_cli) and os.access(env_cli, os.X_OK):
            return env_cli
        which = shutil.which(env_cli)
        return which
    return shutil.which("docker") or shutil.which("podman")


class ContainerInterface:
    """A helper class for managing Isaac Lab containers."""

    def __init__(
        self,
        context_dir: Path,
        profile: str = "base",
        yamls: list[str] | None = None,
        envs: list[str] | None = None,
        statefile: StateFile | None = None,
        suffix: str | None = None,
        container_cli: str | None = None,
    ):
        """Initialize the container interface with the given parameters.

        Args:
            context_dir:
                The context directory for Docker operations.
            profile:
                The profile name for the container. Defaults to "base".
            yamls:
                A list of yaml files to extend ``docker-compose.yaml`` settings. These are extended in the order
                they are provided. Defaults to None, in which case no additional yaml files are added.
            envs:
                A list of environment variable files to extend the ``.env.base`` file. These are extended in the order
                they are provided. Defaults to None, in which case no additional environment variable files are added.
            statefile:
                An instance of the :class:`Statefile` class to manage state variables. Defaults to None, in
                which case a new configuration object is created by reading the configuration file at the path
                ``context_dir/.container.cfg``.
            suffix:
                Optional docker image and container name suffix.  Defaults to None, in which case, the docker name
                suffix is set to the empty string. A hyphen is inserted in between the profile and the suffix if
                the suffix is a nonempty string.  For example, if "base" is passed to profile, and "custom" is
                passed to suffix, then the produced docker image and container will be named ``isaac-lab-base-custom``.
            container_cli:
                Path to the ``docker`` or ``podman`` binary. Defaults to None, in which case
                :func:`resolve_container_cli` is used (``ISAACLAB_CONTAINER_CLI``, then ``docker``, then ``podman``).
        """
        # set the context directory
        self.context_dir = context_dir

        # create a state-file if not provided
        # the state file is a manager of run-time state variables that are saved to a file
        if statefile is None:
            self.statefile = StateFile(path=self.context_dir / ".container.cfg")
        else:
            self.statefile = statefile

        # set the profile and container name
        self.profile = profile
        if self.profile == "isaaclab":
            # Silently correct from isaaclab to base, because isaaclab is a commonly passed arg
            # but not a real profile
            self.profile = "base"

        # set the docker image and container name suffix
        if suffix is None or suffix == "":
            # if no name suffix is given, default to the empty string as the name suffix
            self.suffix = ""
        else:
            # insert a hyphen before the suffix if a suffix is given
            self.suffix = f"-{suffix}"

        # set names for easier reference
        self.base_service_name = "isaac-lab-base"
        self.service_name = f"isaac-lab-{self.profile}"
        self.container_name = f"{self.service_name}{self.suffix}"
        self.image_name = f"{self.service_name}{self.suffix}:latest"

        # keep the environment variables from the current environment,
        # except make sure that the docker name suffix is set from the script
        self.environ = os.environ.copy()
        self.environ["DOCKER_NAME_SUFFIX"] = self.suffix

        self.container_cli = container_cli if container_cli is not None else resolve_container_cli()
        if not self.container_cli:
            raise RuntimeError(
                "Could not resolve a container engine. Install Docker or Podman, or set ISAACLAB_CONTAINER_CLI "
                "to the full path of the executable (e.g. /usr/bin/podman)."
            )

        # detect if we're using podman-compose
        self._detect_compose_command()

        # resolve the image extension through the passed yamls and envs
        self._resolve_image_extension(yamls, envs)
        # load the environment variables from the .env files
        self._parse_dot_vars()

    def print_info(self):
        """Print the container interface information."""
        print("=" * 60)
        print(f"{'DOCKER CONTAINER INFO':^60}")  # Centered title
        print("=" * 60)

        print(f"{'Profile:':25} {self.profile}")
        print(f"{'Suffix:':25} {self.suffix}")
        print(f"{'Container engine:':25} {self.container_cli}")
        print(f"{'Service Name:':25} {self.service_name}")
        print(f"{'Image Name:':25} {self.image_name}")
        print(f"{'Container Name:':25} {self.container_name}")

        print("-" * 60)
        print(f"{'Docker Compose Arguments':^60}")
        print("-" * 60)
        print(f"{'YAMLs:':25} {' '.join(self.add_yamls)}")
        print(f"{'Profiles:':25} {' '.join(self.add_profiles)}")
        print(f"{'Env Files:':25} {' '.join(self.add_env_files)}")
        print("=" * 60)

    """
    Operations.
    """

    def is_container_running(self) -> bool:
        """Check if the container is running.

        Returns:
            True if the container is running, otherwise False.
        """
        status = subprocess.run(
            [self.container_cli, "container", "inspect", "-f", "{{.State.Status}}", self.container_name],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.strip()
        return status == "running"

    def does_image_exist(self) -> bool:
        """Check if the Docker image exists.

        Returns:
            True if the image exists, otherwise False.
        """
        result = subprocess.run([self.container_cli, "image", "inspect", self.image_name], capture_output=True, text=True)
        return result.returncode == 0

    def build(self):
        """Build the Docker image."""
        print("[INFO] Building the docker image for the profile 'base'...\n")
        # build the image for the base profile
        cmd = (
            [self.container_cli, "compose"]
            + ["--file", "docker-compose.yaml"]
            + ["--profile", "base"]
            + ["--env-file", ".env.base"]
            + ["build", self.base_service_name]
        )
        subprocess.run(cmd, check=False, cwd=self.context_dir, env=self.environ)
        print("[INFO] Finished building the docker image for the profile 'base'.\n")

        # build the image for the profile
        if self.profile != "base":
            print(f"[INFO] Building the docker image for the profile '{self.profile}'...\n")
            cmd = (
                [self.container_cli, "compose"]
                + self.add_yamls
                + self.add_profiles
                + self.add_env_files
                + ["build", self.service_name]
            )
            subprocess.run(cmd, check=False, cwd=self.context_dir, env=self.environ)
            print(f"[INFO] Finished building the docker image for the profile '{self.profile}'.\n")

    def start(self):
        """Build and start the Docker container using the Docker compose command."""
        print(
            f"[INFO] Building the docker image and starting the container '{self.container_name}' in the"
            " background...\n"
        )
        # Check if the container history file exists
        container_history_file = self.context_dir / ".isaac-lab-docker-history"
        if not container_history_file.exists():
            # Create the file with sticky bit on the group
            container_history_file.touch(mode=0o2644, exist_ok=True)

        # build the image for the base profile if not running base (up will build base already if profile is base)
        if self.profile != "base":
            if self.is_podman:
                # For podman, specify the base service directly
                subprocess.run(
                    [
                        self.container_cli,
                        "compose",
                        "--file",
                        "docker-compose.yaml",
                        "--env-file",
                        ".env.base",
                        "build",
                        "isaac-lab-base",
                    ],
                    check=False,
                    cwd=self.context_dir,
                    env=self.environ,
                )
            else:
                # For docker compose, use profiles
                subprocess.run(
                    [
                        self.container_cli,
                        "compose",
                        "--file",
                        "docker-compose.yaml",
                        "--profile",
                        "base",
                        "--env-file",
                        ".env.base",
                        "build",
                        self.base_service_name,
                    ],
                    check=False,
                    cwd=self.context_dir,
                    env=self.environ,
                )

        # build the image for the profile
        if self.is_podman and self.service_name:
            # For podman-compose, specify the service name directly
            subprocess.run(
                [
                    self.container_cli,
                    "compose"
                ]
                + self.add_yamls
                + self.add_env_files
                + [
                    "up",
                    "--detach",
                    "--build",
                    "--remove-orphans",
                    self.service_name
                ],
                check=False,
                cwd=self.context_dir,
                env=self.environ,
            )
        else:
            # For docker compose, use profiles
            subprocess.run(
                [self.container_cli, "compose"]
                + self.add_yamls
                + self.add_profiles
                + self.add_env_files
                + ["up", "--detach", "--build", "--remove-orphans"],
                check=False,
                cwd=self.context_dir,
                env=self.environ,
            )

    def restart(self):
        """Restart an existing stopped container without rebuilding.

        Raises:
            RuntimeError: If the container doesn't exist or is already running.
        """
        # Check if container is already running
        if self.is_container_running():
            print(f"[INFO] Container '{self.container_name}' is already running.")
            return

        # Check if container exists (even if stopped)
        result = subprocess.run(
            [self.container_cli, "container", "inspect", "-f", "{{.State.Status}}", self.container_name],
            capture_output=True,
            text=True,
            check=False,
        )
        
        if result.returncode != 0:
            raise RuntimeError(
                f"The container '{self.container_name}' does not exist. "
                "Please use the 'start' command to build and create it first."
            )
        
        status = result.stdout.strip()
        print(f"[INFO] Container '{self.container_name}' is currently '{status}'.")
        print(f"[INFO] Starting the existing container '{self.container_name}' without rebuilding...\n")
        
        # Start the container without rebuilding
        if self.is_podman and self.service_name:
            # For podman-compose, use the service name
            subprocess.run(
                [
                    self.container_cli,
                    "compose"
                ]
                + self.add_yamls
                + self.add_env_files
                + [
                    "start",
                    self.service_name
                ],
                check=False,
                cwd=self.context_dir,
                env=self.environ,
            )
        else:
            # For docker compose, use profiles
            subprocess.run(
                [self.container_cli, "compose"]
                + self.add_yamls
                + self.add_profiles
                + self.add_env_files
                + ["start"],
                check=False,
                cwd=self.context_dir,
                env=self.environ,
            )
        
        print(f"[INFO] Container '{self.container_name}' has been started successfully.")

    def exec_command(self, cmd: str | list[str], workdir: str | None = None):
        """Execute a command in the running container (non-interactive).

        This method is suitable for automation tools, scripts, and LLMs that need to run commands
        and capture output without interactive terminal sessions.

        Args:
            cmd: The command to execute. Can be a string (e.g., "ls -la") or list of strings (e.g., ["ls", "-la"]).
                 If a string is provided, it will be executed via bash -c.
            workdir: Optional working directory inside the container. Defaults to None (uses container's default).

        Raises:
            RuntimeError: If the container is not running.
        """
        if not self.is_container_running():
            raise RuntimeError(
                f"The container '{self.container_name}' is not running. "
                "Use 'restart' to start it or 'start' to build and create it."
            )

        # Convert string command to list for bash -c execution
        if isinstance(cmd, str):
            cmd_display = cmd
            cmd_list = ["bash", "-c", cmd]
        else:
            cmd_display = " ".join(cmd)
            cmd_list = cmd

        print(f"[INFO] Executing command in '{self.container_name}': {cmd_display}\n")
        
        # Build the docker exec command
        exec_cmd = [self.container_cli, "exec"]
        
        # Add DISPLAY environment variable if available (for GUI apps)
        if "DISPLAY" in os.environ:
            exec_cmd.extend(["-e", f"DISPLAY={os.environ['DISPLAY']}"])
        
        # Add working directory if specified
        if workdir:
            exec_cmd.extend(["--workdir", workdir])
        
        # Add container name
        exec_cmd.append(self.container_name)
        
        # Add the command to execute
        exec_cmd.extend(cmd_list)
        
        # Execute the command (output will be shown in terminal)
        result = subprocess.run(exec_cmd, check=False)
        
        # Return the exit code
        return result.returncode

    def enter(self):
        """Enter the running container by executing a bash shell.

        Raises:
            RuntimeError: If the container is not running.
        """
        if self.is_container_running():
            print(f"[INFO] Entering the existing '{self.container_name}' container in a bash session...\n")
            cmd = (
                [self.container_cli, "exec", "--interactive", "--tty"]
                + (["-e", f"DISPLAY={os.environ['DISPLAY']}"] if "DISPLAY" in os.environ else [])
                + [self.container_name, "bash"]
            )
            subprocess.run(cmd)
        else:
            raise RuntimeError(f"The container '{self.container_name}' is not running.")

    def stop(self):
        """Stop the running container using the Docker compose command."""
        if self.is_container_running():
            print(f"[INFO] Stopping the launched docker container '{self.container_name}'...\n")
            if self.is_podman and self.service_name:
                # For podman-compose, specify the service name directly
                subprocess.run(
                    [self.container_cli, "compose"] + self.add_yamls + self.add_env_files + ["down", "--volumes", self.service_name],
                    check=False,
                    cwd=self.context_dir,
                    env=self.environ,
                )
            else:
                # For docker compose, use profiles
                subprocess.run(
                    [self.container_cli, "compose"] + self.add_yamls + self.add_profiles + self.add_env_files + ["down", "--volumes"],
                    check=False,
                    cwd=self.context_dir,
                    env=self.environ,
                )
        else:
            print(
                f"[INFO] Can't stop container '{self.container_name}' as it is not running."
                f" To check if the container is running, run '{Path(self.container_cli).name} ps' or"
                f" '{Path(self.container_cli).name} container ls'.\n"
            )

    def copy(self, output_dir: Path | None = None):
        """Copy artifacts from the running container to the host machine.

        Args:
            output_dir: The directory to copy the artifacts to. Defaults to None, in which case
                the context directory is used.

        Raises:
            RuntimeError: If the container is not running.
        """
        if self.is_container_running():
            print(f"[INFO] Copying artifacts from the '{self.container_name}' container...\n")
            if output_dir is None:
                output_dir = self.context_dir

            # create a directory to store the artifacts
            output_dir = output_dir.joinpath("artifacts")
            if not output_dir.is_dir():
                output_dir.mkdir()

            # define dictionary of mapping from docker container path to host machine path
            docker_isaac_lab_path = Path(self.dot_vars["DOCKER_ISAACLAB_PATH"])
            artifacts = {
                docker_isaac_lab_path.joinpath("logs"): output_dir.joinpath("logs"),
                docker_isaac_lab_path.joinpath("docs/_build"): output_dir.joinpath("docs"),
                docker_isaac_lab_path.joinpath("data_storage"): output_dir.joinpath("data_storage"),
            }
            # print the artifacts to be copied
            for container_path, host_path in artifacts.items():
                print(f"\t -{container_path} -> {host_path}")
            # remove the existing artifacts
            for path in artifacts.values():
                shutil.rmtree(path, ignore_errors=True)

            # copy the artifacts
            for container_path, host_path in artifacts.items():
                cmd = [self.container_cli, "cp", f"{self.container_name}:{container_path}/", host_path]
                subprocess.run(cmd, check=False, cwd=self.context_dir, env=self.environ)
            print("\n[INFO] Finished copying the artifacts from the container.")
        else:
            raise RuntimeError(f"The container '{self.container_name}' is not running.")

    def config(self, output_yaml: Path | None = None):
        """Process the Docker compose configuration based on the passed yamls and environment files.

        If the :attr:`output_yaml` is not None, the configuration is written to the file. Otherwise, it is printed to
        the terminal.

        Args:
            output_yaml: The path to the yaml file where the configuration is written to. Defaults
                to None, in which case the configuration is printed to the terminal.
        """
        print("[INFO] Configuring the passed options into a yaml...\n")

        # resolve the output argument
        if output_yaml is not None:
            output = ["--output", output_yaml]
        else:
            output = []

        # run the docker compose config command to generate the configuration
        if self.is_podman:
            # For podman-compose, don't use profiles
            subprocess.run(
                [self.container_cli, "compose"] + self.add_yamls + self.add_env_files + ["config"] + output,
                check=False,
                cwd=self.context_dir,
                env=self.environ,
            )
        else:
            # For docker compose, use profiles
            subprocess.run(
                [self.container_cli, "compose"] + self.add_yamls + self.add_profiles + self.add_env_files + ["config"] + output,
                check=False,
                cwd=self.context_dir,
                env=self.environ,
            )

    """
    Helper functions.
    """

    def _detect_compose_command(self):
        """Detect whether we're using docker compose or podman-compose and set appropriate flags."""
        try:
            result = subprocess.run(
                [self.container_cli, "compose", "version"],
                capture_output=True,
                text=True,
                check=False,
            )
            combined = ((result.stdout or "") + (result.stderr or "")).lower()
            cli_name = Path(self.container_cli).name.lower()
            self.is_podman = cli_name.startswith("podman") or "podman-compose" in combined
        except (subprocess.SubprocessError, FileNotFoundError):
            # Fallback: assume docker if we can't determine
            self.is_podman = False

    def _resolve_image_extension(self, yamls: list[str] | None = None, envs: list[str] | None = None):
        """Resolve the image extension by setting up YAML files, profiles, and environment files for the
        Docker compose command.

        Args:
            yamls: A list of yaml files to extend ``docker-compose.yaml`` settings. These are extended in the order
                they are provided.
            envs: A list of environment variable files to extend the ``.env.base`` file. These are extended in the order
                they are provided.
        """
        self.add_yamls = ["--file", "docker-compose.yaml"]

        # Handle profiles differently for podman-compose vs docker compose
        if self.is_podman:
            # podman-compose doesn't support --profile, so we'll specify service names directly
            self.add_profiles = []
        else:
            # Standard docker compose supports profiles
            self.add_profiles = ["--profile", f"{self.profile}"]

        self.add_env_files = ["--env-file", ".env.base"]

        # extend env file based on profile
        if self.profile != "base":
            self.add_env_files += ["--env-file", f".env.{self.profile}"]

        # extend the env file based on the passed envs
        if envs is not None:
            for env in envs:
                self.add_env_files += ["--env-file", env]

        # extend the docker-compose.yaml based on the passed yamls
        if yamls is not None:
            for yaml in yamls:
                self.add_yamls += ["--file", yaml]

    def _parse_dot_vars(self):
        """Parse the environment variables from the .env files.

        Based on the passed ".env" files, this function reads the environment variables and stores them in a dictionary.
        The environment variables are read in order and overwritten if there are name conflicts, mimicking the behavior
        of Docker compose.
        """
        self.dot_vars: dict[str, Any] = {}

        # check if the number of arguments is even for the env files
        if len(self.add_env_files) % 2 != 0:
            raise RuntimeError(
                "The parameters for env files are configured incorrectly. There should be an even number of arguments."
                f" Received: {self.add_env_files}."
            )

        # read the environment variables from the .env files
        for i in range(1, len(self.add_env_files), 2):
            with open(self.context_dir / self.add_env_files[i]) as f:
                self.dot_vars.update(dict(line.strip().split("=", 1) for line in f if "=" in line))
        
        # Merge the .env file variables into the environment for podman-compose compatibility
        # This ensures that podman-compose has access to all environment variables from .env files
        self.environ.update(self.dot_vars)