package main

import (
	"encoding/json"
	"image/color"
	"io"
	"log"
	"os"
	"path/filepath"
	"testing"

	sm "github.com/flopp/go-staticmaps"
	"github.com/fogleman/gg"
	"github.com/golang/geo/s2"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/geojson"
	"github.com/paulmach/orb/planar"
	"github.com/paulmach/orb/simplify"
)

var testdataRootPath = filepath.Join(".", "testdata")

func testdataReader(geojsonFile string) (io.Reader, error) {
	return os.Open(filepath.Join(testdataRootPath, geojsonFile))
}

func writeMap(f *geojson.Feature, pathto string) error {
	ctx := sm.NewContext()
	ctx.SetSize(800, 800)

	switch g := f.Geometry.(type) {
	case orb.LineString:
		lls := []s2.LatLng{}
		for _, v := range g {
			lls = append(lls, s2.LatLngFromDegrees(v[1], v[0]))
		}
		v := sm.NewPath(lls, color.RGBA{0, 0, 255, 255}, 3)
		ctx.AddObject(v)
	}

	img, err := ctx.Render()
	if err != nil {
		return err
	}

	os.MkdirAll(filepath.Dir(pathto), 0777)
	if err := gg.SavePNG(pathto, img); err != nil {
		return err
	}
	return nil
}

func TestReadStreamToLineString(t *testing.T) {
	in, err := testdataReader("edge.json")
	if err != nil {
		t.Fatal(err)
	}
	f, err := readStreamToLineString(in)
	if err != nil {
		t.Fatal(err)
	}

	// j, err := json.MarshalIndent(f, "", "  ")
	j, err := json.Marshal(f)
	if err != nil {
		t.Fatal(err)
	}
	log.Println(string(j))

	if err := writeMap(f, filepath.Join(testdataRootPath, "output", "edge_original.png")); err != nil {
		t.Fatal(err)
	}

	lsSimpler := f.Geometry.(orb.LineString).Clone()

	// Simplify
	threshold := planar.Distance(orb.Point{0, 0}, orb.Point{0, 0.00006})
	log.Printf("threshold: %f", threshold)
	simplifier := simplify.DouglasPeucker(float64(6) * earthCircumferenceDegreesPerMeter)
	lsSimpler = simplifier.Simplify(lsSimpler).(orb.LineString)

	// j, err := json.MarshalIndent(f, "", "  ")
	j, err = json.Marshal(lsSimpler)
	if err != nil {
		t.Fatal(err)
	}
	log.Println(string(j))

	distance := planar.Distance(f.Geometry.Bound().Min, f.Geometry.Bound().Max)
	t.Logf("distance: %f", distance)
	metersQ := earthCircumference * distance
	t.Logf("metersQ: %f", metersQ)

	if err := writeMap(geojson.NewFeature(lsSimpler), filepath.Join(testdataRootPath, "output", "edge_simplified.png")); err != nil {
		t.Fatal(err)
	}

	// KFilter

	lsKalman := f.Geometry.(orb.LineString).Clone()
	lsKalmanF := geojson.NewFeature(lsKalman)
	for k, v := range f.Properties {
		lsKalmanF.Properties[k] = v
	}
	if err := modifyKalman(lsKalmanF); err != nil {
		t.Fatal(err)
	}

	if err := writeMap(lsKalmanF, filepath.Join(testdataRootPath, "output", "edge_kalman.png")); err != nil {
		t.Fatal(err)
	}
}
