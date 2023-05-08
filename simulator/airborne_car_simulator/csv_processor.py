import argparse
import csv
import matplotlib.pyplot as plt


def import_csv(csvfilename: str):
    data = []
    with open(csvfilename, "r", encoding="utf-8", errors="ignore") as scraped:
        reader = csv.reader(scraped, delimiter=',')
        for row in reader:
            if row:  # avoid blank lines
                columns = [row[0], row[1], row[2], row[3], row[4]]
                data.append(columns)
    scraped.close()
    return data


def read_messages(vel_csv: str, pitch_csv: str, motor_csv: str):
    """
    :param vel_csv: path to velocity csv from foxglove; data from '/drive.drive.speed'
    :param pitch_csv: path to pitch_error csv; data from '/pitch_error'
    :param motor_csv: path to motor_speed csv; data from '/commands/motor/speed.data'
    :return:
    """
    pitch_data = import_csv(pitch_csv)
    vel_data = import_csv(vel_csv)
    motor_data = import_csv(motor_csv)
    start_time = None
    end_time = None
    # Align timestamps
    start_time = pitch_data[1][0]
    end_time = pitch_data[-1][0]
    print(f'start time: {start_time}, end time: {end_time}')
    # Plot pitch
    pitch_values = []
    for i, row in enumerate(pitch_data):
        if i == 0:
            continue
        else:
            pitch_values.append(round(float(row[-1]), 6))
    # Plot velocity
    vel_values = []
    elapsed_time = []
    first_time = None
    flag = False
    for i, row in enumerate(vel_data):
        if i == 0:
            continue
        else:
            if float(start_time) - 0.01 <= float(row[0]) <= float(end_time) + 0.01:
                if not flag:
                    first_time = row[0]
                    flag = True
                vel_values.append(round(float(row[-1]), 6))
                elapsed_time.append(float(row[0]) - float(first_time))
    # Plot motor speed
    motor_values = []
    for i, row in enumerate(motor_data):
        if i == 0:
            continue
        else:

            if float(start_time) - 0.01 <= float(row[0]) <= float(end_time) + 0.01:
                print(row[0])
                motor_values.append(round(float(row[-1]), 6))
    print(
        f'pitch len: {len(pitch_data[1:])}, vel len: {len(vel_values)}, motor len: {len(motor_values)}')
    fig, axs = plt.subplots(3, sharex=True, sharey=False)
    fig.suptitle('Experiment plots')
    axs[0].plot(elapsed_time, pitch_values)
    axs[0].set_title('Pitch error vs. t')
    axs[0].set_ylabel('pitch error (rad)')
    axs[1].plot(elapsed_time, vel_values, 'tab:orange')
    axs[1].set_title('Velocity vs. t')
    axs[1].set_ylabel('velocity (m/s)')
    axs[2].plot(elapsed_time, motor_values, 'tab:green')
    axs[2].set_title('Motor speed vs. t')
    axs[2].set_ylabel('motor speed (revolutions per minute)')
    axs[2].set_xlabel('t (s)')
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "pitch_error_csv", help="csv path of pitch error to extract timestamps"
    )
    parser.add_argument(
        "motor_csv", help="motor csv"
    )

    args = parser.parse_args()
    read_messages(args.input, args.pitch_error_csv, args.motor_csv)
