import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
import rosbag2_py
import matplotlib.pyplot as plt 

def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        print(topic)
        if topic == '/pitch_error':
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
    del reader

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )

    args = parser.parse_args()
    recorded_pitch = []
    timestamps = []
    for topic, msg, timestamp in read_messages(args.input):
        if isinstance(msg, String):
            print(f"{topic} [{timestamp}]: '{msg.data}'")
            recorded_pitch.append(msg.data)
            timestamps.append(timestamp)

        else:
            print(f"{topic} [{timestamp}]: ({type(msg).__name__})")


    plt.plot(timestamps, recorded_pitch)
    plt.title('pitch angles vs. t')
    plt.ylabel('Pitch angle')
    plt.xlabel('t')
    plt.show()
    
if __name__ == "__main__":
    main()