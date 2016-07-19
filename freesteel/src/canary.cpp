#include <pits/CoreRoughGeneration.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <bolts/I1.h>

void OutputDebugStringG(const char* str)
{
    std::cerr << str << "\n";
}

void OutputDebugStringG(const char* str0, const char* strf, int line1)
{
    std::cerr << strf << ":" << line1 << " " << str0 << "\n";
}

SurfX ReadOFF(const std::string& off) {
    std::ifstream is(off);
    is.exceptions(std::ifstream::failbit);

    std::string line;
    std::getline(is, line);
    if (line != "OFF")
        throw std::runtime_error("Invalid OFF format");
    unsigned nvertices;
    unsigned nfaces;
    unsigned nedges;

    is >> nvertices >> nfaces >> nedges;

    struct vec3 {
        double x;
        double y;
        double z;
    };

    std::vector<vec3> vertices;
    for (unsigned i = 0; i < nvertices; ++i) {
        vec3 v;
        is >> v.x >> v.y >> v.z;
        vertices.push_back(v);
    }

    SurfXBuilder builder;
    for (unsigned i = 0; i < nfaces; ++i) {
        unsigned n;
        is >> n;
        if (n != 3)
            throw std::runtime_error("Invalid OFF format - triangles only");
        unsigned a;
        unsigned b;
        unsigned c;
        is >> a >> b >> c;
        builder.PushTriangle(
            {vertices[a].x, vertices[a].y, vertices[a].z},
            {vertices[b].x, vertices[b].y, vertices[b].z},
            {vertices[c].x, vertices[c].y, vertices[c].z});
    }
    return builder.Build();
}

PathXSeries MakeRectBoundary(const I1& xrg, const I1& yrg, double z)
{
    PathXSeries bound(z);
    bound.Add(P2(xrg.lo, yrg.lo));
    bound.Add(P2(xrg.hi, yrg.lo));
    bound.Add(P2(xrg.hi, yrg.hi));
    bound.Add(P2(xrg.lo, yrg.hi));
    bound.Add(P2(xrg.lo, yrg.lo));
    bound.Break();
    return bound;
}

int usage()
{
    std::cout << "./canary input.off\n";
    return 0;
}

template<class InputIt, class BinaryOperation>
void transform_adjacent(InputIt first, InputIt last, BinaryOperation op) {
    if (first == last) return;

    auto acc = *first;
    while (++first != last) {
        auto val = *first;
        op(acc, val);
        acc = std::move(val);
    }
}


int main(int argc, char* argv[]) {

    std::vector<std::string> args(argv, argv+argc);
    args.erase(begin(args));

    if (args.empty()) return usage();

    auto sx = ReadOFF(args[0]);
    auto bound = MakeRectBoundary(sx.gxrg, sx.gyrg, sx.gzrg.hi + 1);

    std::cerr << "bbox "
              << "min (" << sx.gxrg.lo << ", " << sx.gyrg.lo << ", " << sx.gzrg.lo << ") "
              << "max (" << sx.gxrg.hi << ", " << sx.gyrg.hi << ", " << sx.gzrg.hi << ") "
              << "\n";

    double cr = 3.;
    double fr = 0.;
    double sd = 1.;
    MachineParams params;
    // linking parameters
        params.leadoffdz = 0.1;
        params.leadofflen = 1.1;
        params.leadoffrad = 2.0;
        params.retractzheight = sx.gzrg.hi + 5.0;
        params.leadoffsamplestep = 0.6;

    // cutting parameters
        params.toolcornerrad = cr;
        params.toolflatrad = fr;
        params.samplestep = 0.4;
        params.stepdown = sd;
        params.clearcuspheight = sd / 3.0;

    // weave parameters
        params.triangleweaveres = 0.51;
        params.flatradweaveres = 0.71;

    // stearing parameters
    // fixed values controlling the step-forward of the tool and
    // changes of direction.
        params.dchangright = 0.17;
        params.dchangrightoncontour = 0.37;
        params.dchangleft = -0.41;
        params.dchangefreespace = -0.6;
        params.sidecutdisplch = 0.0;
        params.fcut = 1000;
        params.fretract = 5000;
        params.thintol = 0.0001;


    auto tp = MakeCorerough(sx, bound, params);

    for (auto& path : tp)
    {
        auto brks = path.brks;
        if (brks.empty() || brks[0] != 0)
            brks.insert(brks.begin(), 0);

        if (brks.back() != path.pths.size())
            brks.push_back(path.pths.size());

        unsigned link_path = 0;
        transform_adjacent(begin(brks), end(brks), [&](std::size_t a, std::size_t b) {
            for (std::size_t i = a; i < b; ++i)
            {
                std::cout << std::fixed << "G01";
                std::cout << " X" << path.GetX(i);
                std::cout << " Y" << path.GetY(i);
                std::cout << " Z" << path.z;
                std::cout << " F" << params.fcut;
                std::cout << "\n";
            }

            if (link_path < path.GetNbrks())    // No link for last path
            {
                for (unsigned i = 0; i < path.GetNlnks(link_path); ++i)
                    std::cout << std::fixed << "G00 " << "X" << path.GetLinkX(link_path, i) << " Y" << path.GetLinkY(link_path, i) << " Z" << path.GetLinkZ(link_path, i) << "\n";
                ++link_path;
            }
        });
        std::cout << "\n";
    }
}

