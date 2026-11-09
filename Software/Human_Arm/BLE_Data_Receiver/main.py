from __future__ import annotations
import subprocess
import sys

def run_command(arguments: list[str]) -> None:
    """    Run one of the BLE tools using the same Python interpreter that started this script."""
    command = [
        sys.executable,
        "-m",
        "aurora.recording.record_ble_sensors",
        *arguments,
    ]

    try:
        subprocess.run(
            command,
            check=False,
        )

    except KeyboardInterrupt:
        print("\nStopped by user.")

def show_menu() -> None:
    print()
    print("======================================")
    print(" AURORA BLE Sensor Tool")
    print("======================================")
    print()
    print("1. Preview live sensor data")
    print("2. Record sensor data")
    print("3. Plot saved recording")
    print("4. Exit")
    print()


def main() -> int:
    while True:
        show_menu()
        choice = input("Select mode: ").strip()
        
        # -----------------------------------------------------
        # Preview
        # -----------------------------------------------------
        if choice == "1":
            print()
            print("Starting live BLE preview...")
            print()
            run_command(["--preview"])

        # -----------------------------------------------------
        # Record
        # -----------------------------------------------------
        elif choice == "2":
            print()
            print("Starting BLE recording...")
            print()
            run_command([])
     
        # -----------------------------------------------------
        # Plot saved recording
        # -----------------------------------------------------  
        elif choice == "3":
            print()
            print("Opening saved recording...")
            print()
            command = [
                sys.executable,
                "-m",
                "aurora.recording.plot_recording",
            ]

            subprocess.run(
                command,
                check=False,
            )
        # -----------------------------------------------------
        # Exit
        # -----------------------------------------------------
        elif choice == "4":
            print()
            print("Exiting AURORA BLE Sensor Tool.")
            return 0

        # -----------------------------------------------------
        # Invalid input
        # -----------------------------------------------------
        else:
            print()
            print("Invalid selection." "Choose 1, 2 or 3.")

if __name__ == "__main__":
    raise SystemExit(
        main()
    )