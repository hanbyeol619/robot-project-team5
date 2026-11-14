
import cv2
import csv

# 기존 ROI 이미지
image_path = "/home/jun/ros2_ws/work_area.png"

# 원본 영상에서 ROI 시작점
ROI_X = 465
ROI_Y = 56

image = cv2.imread(image_path)

if image is None:
    raise RuntimeError("ROI 이미지 파일을 찾을 수 없습니다.")

# 마우스로 객체별 사각형 선택
boxes = cv2.selectROIs(
    "Select Objects",
    image,
    showCrosshair=True,
    fromCenter=False
)

cv2.destroyAllWindows()

result = image.copy()
data = []

for i, (x, y, w, h) in enumerate(boxes, start=1):

    cx = int(x + w / 2)
    cy = int(y + h / 2)

    # 원본 영상 기준 좌표
    full_x = cx + ROI_X
    full_y = cy + ROI_Y

    cv2.rectangle(
        result,
        (x, y),
        (x+w, y+h),
        (0, 255, 0),
        2
    )

    cv2.circle(
        result,
        (cx, cy),
        5,
        (0, 0, 255),
        -1
    )

    cv2.putText(
        result,
        f"ID{i} ({cx},{cy})",
        (x, max(15, y-8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 0, 255),
        1
    )

    data.append([i, cx, cy, full_x, full_y])

    print(
        f"Object {i}: ROI=({cx},{cy}), "
        f"Full=({full_x},{full_y})"
    )

with open(
    "/home/jun/ros2_ws/object_positions.csv",
    "w",
    newline=""
) as f:
    writer = csv.writer(f)
    writer.writerow([
        "id", "roi_x", "roi_y",
        "full_x", "full_y"
    ])
    writer.writerows(data)

cv2.imwrite(
    "/home/jun/ros2_ws/objects_preview.png",
    result
)

cv2.imshow("Object Positions", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
