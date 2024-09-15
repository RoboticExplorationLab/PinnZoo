using Pkg; Pkg.activate(@__DIR__)
using Documenter
include(joinpath(@__DIR__, "../src/PinnZoo.jl")) # Make sure we load local version
using Main.PinnZoo

makedocs(sitename="PinnZoo Documentation")