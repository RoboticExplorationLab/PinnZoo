pkg_dir = @__DIR__  # This is YourPackage/deps
root_dir = joinpath(pkg_dir, "..")
build_dir = joinpath(@__DIR__, "../build")

# Create build directory if it doesn't exist
isdir(build_dir) || mkpath(build_dir)

# Run cmake and build
cd(build_dir) do
    run(pipeline(`cmake ..`, stdout=stdout, stderr=stderr))
    run(pipeline(`cmake --build . -- -j`, stdout=stdout, stderr=stderr))
end

println("Build completed.")