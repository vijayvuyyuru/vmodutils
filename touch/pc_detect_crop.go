package touch

import (
	"context"
	"fmt"
	"image"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/vision/objectdetection"

	"github.com/erh/vmodutils"
)

var DetectCropCameraModel = vmodutils.NamespaceFamily.WithModel("pc-detect-crop-camera")

func init() {
	resource.RegisterComponent(
		camera.API,
		DetectCropCameraModel,
		resource.Registration[camera.Camera, *DetectCropCameraConfig]{
			Constructor: newDetectCropCamera,
		})
}

type DetectCropCameraConfig struct {
	Src     string
	Service string

	Labels []string
	Min    float64
}

func (dccc *DetectCropCameraConfig) Validate(path string) ([]string, []string, error) {
	if dccc.Src == "" {
		return nil, nil, fmt.Errorf("need a src camera")
	}
	if dccc.Service == "" {
		return nil, nil, fmt.Errorf("need a service")
	}

	return []string{dccc.Src, dccc.Service}, nil, nil
}

type detectCropCamera struct {
	resource.AlwaysRebuild

	name   resource.Name
	cfg    *DetectCropCameraConfig
	logger logging.Logger

	src     camera.Camera
	service vision.Service
}

func newDetectCropCamera(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*DetectCropCameraConfig](config)
	if err != nil {
		return nil, err
	}

	cc := &detectCropCamera{
		name:   config.ResourceName(),
		cfg:    newConf,
		logger: logger,
	}

	cc.src, err = camera.FromProvider(deps, newConf.Src)
	if err != nil {
		return nil, err
	}

	cc.service, err = vision.FromProvider(deps, newConf.Service)
	if err != nil {
		return nil, err
	}

	props, err := cc.src.Properties(ctx)
	if err != nil {
		return nil, err
	}
	if props.IntrinsicParams == nil {
		return nil, fmt.Errorf("no IntrinsicParams on %s", newConf.Src)
	}

	return nil, fmt.Errorf("finish me")
	//return cc, nil
}

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
