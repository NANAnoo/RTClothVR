#pragma once

#include <functional>

namespace RTCloth
{
    struct AABB
    {
        bool Contains(FVector const& Pos) const
        {
            return Pos[0] < Max[0] && Pos[0] > Min[0] &&
               Pos[1] < Max[1] && Pos[1] > Min[1] &&
               Pos[2] < Max[2] && Pos[2] > Min[2];
        }
        bool Intersect(AABB const&other) const
        {
            return Contains(other.Max) || Contains(other.Min) || other.Contains(Max) || other.Contains(Min);
        }
        AABB operator+(AABB const&Other)
        {
            AABB Result;
            Result.Max[0] = std::max(Max[0], Other.Max[0]);
            Result.Max[1] = std::max(Max[1], Other.Max[1]);
            Result.Max[2] = std::max(Max[2], Other.Max[2]);

            Result.Min[0] = std::min(Min[0], Other.Min[0]);
            Result.Min[1] = std::min(Min[1], Other.Min[1]);
            Result.Min[2] = std::min(Min[2], Other.Min[2]);

            return Result;
        }
        FVector Max;
        FVector Min;
    };

    template<typename T, typename AABB_builder>
    struct BVH_Node
    {
        constexpr uint32 static Size()
        {
            return sizeof(BVH_Node<T, AABB_builder>);
        }
        explicit BVH_Node(T const& Obj) : value(Obj), box(AABB_builder{}(Obj))
        {
        } 
        explicit BVH_Node(AABB const& Box)
        {
            box = Box;
        }

        static int BuildFrom(TArray<T> const&Objects, TArray<BVH_Node<T, AABB_builder>>&Result, int Begin, int End)
        {
            if (End == Begin)
            {
                BVH_Node NewNode(Objects[Begin]);
                Result.Add(NewNode);
            } else
            {
                int Mid = (Begin + End) >> 1;
                int L = BuildFrom(Objects, Result, Begin, Mid);
                int R = BuildFrom(Objects, Result, Mid + 1, End);
                BVH_Node NewNode(Result[L].box + Result[R].box);
                NewNode.L = L;
                NewNode.R = R;
                Result.Add(NewNode);
            }
            return Result.Num() - 1;
        }
    
        void Intersect(FVector const&Pos, TArray<BVH_Node> const& Nodes, std::function<void(T const&Value)> const&Handler) const
        {
            if (box.Contains(Pos))
            {
                if (L == -1 && R == -1)
                {
                    Handler(value);
                } else
                {
                    if (L >= 0)
                        Nodes[L].Intersect(Pos, Nodes, Handler);
                    if (R >= 0)
                        Nodes[R].Intersect(Pos, Nodes, Handler);
                }
            
            }
        }
        T value;
        AABB box;
        int L = -1;
        int R = -1;
    };
}