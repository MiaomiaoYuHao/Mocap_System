#include "detect/CentroidDetector.hpp"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <vector>
using namespace mocap;

int main(){
    const int w=64,h=48;
    std::vector<uint8_t> img(w*h,0);
    // 放两个高斯亮斑，真值坐标带小数
    struct P{double cx,cy;} truth[2]={{20.3,15.7},{45.6,30.2}};
    auto put=[&](double cx,double cy){
        for(int y=0;y<h;++y)for(int x=0;x<w;++x){
            double dx=x-cx,dy=y-cy,s2=2*1.5*1.5;
            int v=int(240*std::exp(-(dx*dx+dy*dy)/s2));
            int nv=img[y*w+x]+v; img[y*w+x]=(uint8_t)(nv>255?255:nv);
        }
    };
    put(truth[0].cx,truth[0].cy); put(truth[1].cx,truth[1].cy);

    CentroidDetector det;
    DetectParams p; p.threshold=50; p.minArea=3;
    auto blobs=det.detect(img.data(),w,h,p);
    printf("detected %zu blobs\n",blobs.size());
    assert(blobs.size()==2);
    // 匹配最近真值，检查亚像素误差
    double maxerr=0;
    for(auto&b:blobs){
        double best=1e9; 
        for(auto&t:truth){double e=std::hypot(b.cx-t.cx,b.cy-t.cy); best=std::min(best,e);}
        printf("  blob (%.3f,%.3f) area=%d peak=%.0f  err=%.4fpx\n",b.cx,b.cy,b.area,b.peak,best);
        maxerr=std::max(maxerr,best);
    }
    printf("max subpixel err = %.4f px\n",maxerr);
    assert(maxerr<0.1);
    printf("centroid: ALL PASS\n");
    return 0;
}
