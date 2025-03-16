import argparse
from pg_thread import PoseGraphThread
from visualize.posehub_gui import main_gui


def main(args):
    # Create the pose worker thread and start it
    pg_thread = PoseGraphThread(args)
    pg_thread.start()

    try:
        # Launch the PyQt GUI in the main thread
        main_gui(pg_thread)
    except KeyboardInterrupt:
        pg_thread.stop()
    finally:
        pg_thread.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Python script for running the AR tool tracking with PyQt visualization",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--sub_ip_1",
        default="10.0.0.108",
        type=str,
        help="subscriber ip address sensor 1",
    )
    parser.add_argument(
        "--sub_topic",
        default=["artool", "reference_1", "phantom"],
        type=str,
        help="subscriber topics",
    )
    parser.add_argument(
        "--pub_topic",
        default=["artool", "reference_1", "phantom"],
        type=str,
        help="publisher topics",
    )
    args = parser.parse_args()
    main(args)
