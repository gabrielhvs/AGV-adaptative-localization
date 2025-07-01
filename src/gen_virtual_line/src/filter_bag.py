#!/usr/bin/env python3
import rosbag
import sys
from collections import defaultdict

fraction = 0

def remove_repeated_tf(input_bag, output_bag):
    print(f"Processando: {input_bag} → {output_bag}")

    seen_timestamps = defaultdict(set)  # frame_id → set de timestamps
    i = 0
    fraction = 0
    inpbag = rosbag.Bag(input_bag)
    amountMsg = inpbag.get_message_count()
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in inpbag.read_messages():
            i = i+1
            fraction = progress_bar(fraction, i, amountMsg)
            if topic in ["/tf", "/tf_static"]:
                new_transforms = []

                for transform in msg.transforms:
                    frame = transform.child_frame_id
                    stamp = transform.header.stamp.to_nsec()

                    if stamp not in seen_timestamps[frame]:
                        seen_timestamps[frame].add(stamp)
                        new_transforms.append(transform)
                    else:
                        # TF duplicado encontrado e ignorado
                        pass

                if new_transforms:
                    msg.transforms[:] = new_transforms
                    outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)

    print("✔️  Bag filtrado com sucesso!")


def progress_bar(last_frac, current, total, bar_length=30):
        fraction = 0
        if(last_frac == 1.0):
            print('✔️')
            fraction = last_frac+1
        elif(last_frac < 1.0):
            fraction = float(current) / float(total)
            filled_length = int(bar_length * fraction)
            bar = '▓' * filled_length + '░' * (bar_length - filled_length)
            percent = int(fraction * 100)
            print(f'\r[{bar}] {percent}%', end='', flush=True)
        return fraction

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Uso: python remove_tf_repeated.py input.bag output.bag")
        sys.exit(1)

    input_bag = sys.argv[1]
    output_bag = sys.argv[2]
    remove_repeated_tf(input_bag, output_bag)
