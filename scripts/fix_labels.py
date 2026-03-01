import os

def convert_to_bbox_only(label_path):
    with open(label_path, 'r') as f:
        lines = f.readlines()
    
    new_lines = []
    for line in lines:
        parts = line.strip().split()
        if len(parts) > 5:
            class_id = parts[0]
            coords = list(map(float, parts[1:]))
            x_coords = coords[::2]
            y_coords = coords[1::2]
            x_center = (min(x_coords) + max(x_coords)) / 2
            y_center = (min(y_coords) + max(y_coords)) / 2
            width = max(x_coords) - min(x_coords)
            height = max(y_coords) - min(y_coords)
            new_lines.append(f"{class_id} {x_center} {y_center} {width} {height}\n")
        else:
            new_lines.append(line)
    
    with open(label_path, 'w') as f:
        f.writelines(new_lines)

if __name__ == '__main__':
    base_path = 'Yolo model5/rebalanced'
    for split in ['train', 'val', 'test']:
        labels_dir = os.path.join(base_path, split, 'labels')
        if os.path.exists(labels_dir):
            for label_file in os.listdir(labels_dir):
                if label_file.endswith('.txt'):
                    convert_to_bbox_only(os.path.join(labels_dir, label_file))
            print(f"Converted {split} labels to bbox format")
