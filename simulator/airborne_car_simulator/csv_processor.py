import argparse
import csv
import matplotlib.pyplot as plt


def read_messages(input_csv: str, pitch_csv: str):
    pitch_values = []
    elapsed_time = []
    # with open(pitch_csv) as pitch_file:
    #     csv_reader = csv.DictReader(pitch_file, delimiter=','):
    #
    with open(input_csv) as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        for row in csv_reader:
            pitch_values.append(row["value"])
            elapsed_time.append(row["elapsed time"])
    print(pitch_values[0], elapsed_time[0])
    plt.plot(elapsed_time, pitch_values)
    plt.title('pitch angles vs. t')
    plt.ylabel('Pitch angle')
    plt.xlabel('t')
    plt.show()
    csv_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )
    parser.add_argument(
        "pitch_error_csv", help="csv path of pitch error to extract timestamps"
    )

    args = parser.parse_args()
    read_messages(args.input)
