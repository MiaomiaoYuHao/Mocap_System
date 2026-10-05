#include <QAbstractVideoBuffer>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QVideoFrame>
#include <QVideoFrameFormat>
#include <cstdio>
#include <memory>
#include <vector>

class Buf : public QAbstractVideoBuffer {
public:
    Buf(const QVideoFrameFormat& f, uchar* y, int ys, uchar* uv, int us) : f_(f) {
        m_.planeCount = 2;
        m_.data[0] = y; m_.bytesPerLine[0] = ys; m_.dataSize[0] = ys * f_.frameHeight();
        m_.data[1] = uv; m_.bytesPerLine[1] = us; m_.dataSize[1] = us * ((f_.frameHeight()+1)/2);
    }
    MapData map(QVideoFrame::MapMode) override { return m_; }
    QVideoFrameFormat format() const override { return f_; }
private:
    QVideoFrameFormat f_;
    MapData m_{};
};

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    const int W = 2560, H = 1440;
    std::vector<uchar> y(W*H, 128), uv(W*(H/2), 128);
    QVideoFrameFormat f(QSize(W,H), QVideoFrameFormat::Format_NV12);
    f.setScanLineDirection(QVideoFrameFormat::TopToBottom);
    QElapsedTimer t; t.start();
    const int N = 10;
    int n = 0;
    for (int i = 0; i < N; ++i) {
        QVideoFrame frame(std::make_unique<Buf>(f, y.data(), W, uv.data(), W));
        QImage img = frame.toImage();
        if (!img.isNull()) ++n;
    }
    const double ms = double(t.nsecsElapsed()) / 1e6 / N;
    std::printf("ok=%d avg_ms=%.2f\n", n, ms);
    return n == N ? 0 : 2;
}
