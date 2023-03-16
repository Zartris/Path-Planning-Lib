/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

#include <vector>
#include <functional>
#include <set>
#include <chrono>
#include <iostream>

namespace AStar {
    struct Vec2i {
        int x, y;

        bool operator==(const Vec2i &coordinates_) const;
    };

    struct Path {
        std::vector<std::vector<int>> path;
        bool solution_found;
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
//    using CoordinateList = std::vector<std::vector<int>>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(Vec2i coord_, Node *parent_ = nullptr);

        uint getScore() const;
    };

    using NodeSet = std::vector<Node *>;

    class Generator {
        bool detectCollision(Vec2i coordinates_);

        Node *findNodeOnList(NodeSet &nodes_, Vec2i coordinates_);

        void releaseNodes(NodeSet &nodes_);

    public:
//        Generator();

        explicit Generator(const std::vector<std::vector<int>> &gridmap);

        void setWorldSize(Vec2i worldSize_);

        void setDiagonalMovement(bool enable_);

        void setHeuristic(HeuristicFunction heuristic_);

        Path findPath(Vec2i source_, Vec2i target_);

        void addCollision(Vec2i coordinates_);

        void removeCollision(Vec2i coordinates_);

        void clearCollisions();

        void setVerbose(bool verbose_);

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
        bool verbose = true;
        bool foundPath = false;
    };

    class Heuristic {
        static Vec2i getDelta(Vec2i source_, Vec2i target_);

    public:
        static uint manhattan(Vec2i source_, Vec2i target_);

        static uint euclidean(Vec2i source_, Vec2i target_);

        static uint octagonal(Vec2i source_, Vec2i target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
