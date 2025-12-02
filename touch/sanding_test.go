package touch

import (
	"image/png"
	"os"
	"testing"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
)

func TestSanding1(t *testing.T) {
	in, err := pointcloud.NewFromFile("data/sanding2.pcd", "")
	test.That(t, err, test.ShouldBeNil)

	look, err := PCLookAtSegment(in)
	test.That(t, err, test.ShouldBeNil)

	img := PCToImage(look)

	file, err := os.Create("sanding2.png")
	test.That(t, err, test.ShouldBeNil)
	defer file.Close()

	err = png.Encode(file, img)
	test.That(t, err, test.ShouldBeNil)

}
