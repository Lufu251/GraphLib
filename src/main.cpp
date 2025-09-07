#include <raylib.h>

#include <graph.hpp>

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;
    // Enable High DPI scaling
    SetConfigFlags(FLAG_WINDOW_HIGHDPI);
    InitWindow(screenWidth, screenHeight, "raylib quick start");

    // Undirected Graph
    std::cout << "\n--- Undirected Graph Example ---\n";
    Graph undirectedGraph(4, false); // 4 nodes (0 to 3), undirected

    undirectedGraph.addEdge(0, 1, 5);
    undirectedGraph.addEdge(0, 2, 8);
    undirectedGraph.addEdge(1, 2, 2);
    undirectedGraph.addEdge(1, 3, 9);
    undirectedGraph.addEdge(2, 3, 6);

    undirectedGraph.printGraph();

    Dijkstra pathSearcher;
    pathSearcher.solve(0, undirectedGraph);
    
    std::vector<NodeID> path = pathSearcher.mesh.constructPath(3);
    for(size_t i=0; i < path.size(); i++){
        std::cout << path[i] << "";
    }
    std::cout << std::endl;
    

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        // TODO: Update your variables here
        //----------------------------------------------------------------------------------

        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();

            ClearBackground(DARKBLUE);

            DrawText("Perfect! This is your first raylib window!", 190, 200, 20, LIGHTGRAY);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}