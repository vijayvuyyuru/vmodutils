package touch

import (
	"fmt"
	"image"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/vision/objectdetection"
)

func PCDetectCrop(
	pc pointcloud.PointCloud,
	detections []objectdetection.Detection,
	props camera.Properties) (pointcloud.PointCloud, error) {

	if props.IntrinsicParams == nil {
		return nil, fmt.Errorf("intrinsics cannot be null")
	}

	boxes := []*image.Rectangle{}
	for _, d := range detections {
		boxes = append(boxes, d.BoundingBox())
	}

	out := pointcloud.NewBasicEmpty()

	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		x, y := props.IntrinsicParams.PointToPixel(p.X, p.Y, p.Z)

		inBox := false
		for _, b := range boxes {
			if int(x) >= b.Min.X &&
				int(x) <= b.Max.X &&
				int(y) >= b.Min.Y &&
				int(y) <= b.Max.Y {
				inBox = true
				break
			}
		}
		if inBox {
			out.Set(p, d)
		}

		return true
	})

	return out, nil
}
