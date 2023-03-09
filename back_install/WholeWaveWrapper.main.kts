@file:DependsOn("info.laht.sspgen:dsl:0.5.1")

import no.ntnu.ihb.sspgen.dsl.ssp
import kotlin.math.PI

ssp("WholeWaveWrapper") {

    resources {
        file("WholeWaveWrapperFMU.fmu")
    }

    ssd("WholeWaveWrapper") {
        description = "s motion with v"
        author = "Shuai Yuan"

        system("WholeWaveWrapper") {

            elements {

                component("WholeWaveWrapperFMU", "proxyfmu://localhost?file=resources/WholeWaveWrapperFMU.fmu") {
                    connectors {

                        real("Xcv", output)
                        real("Ycv", output)
                        real("Zcv", output)
                        real("Xc", output)
                        real("Yc", output)
                        real("Zc", output)
                        real("RXcv", output)
                        real("RYcv", output)
                        real("RZcv", output)
                        real("RXc", output)
                        real("RYc", output)
                        real("RZc", output)
                        real("GlobalAppliedForceXc1", output)
                        real("GlobalAppliedForceYc1", output)
                        real("GlobalAppliedForceZc1", output)

                        real("GlobalAppliedForceXc2", output)
                        real("GlobalAppliedForceYc2", output)
                        real("GlobalAppliedForceZc2", output)
                        
                    }
                    parameterBindings {
                        parameterSet("initialValues") {

                            real("timeStep", 0.05)
                            real("size", 300)
                            real("amplitude", 0)
                            real("period", 8)
                            real("heading", 0)
                        }
                    }
                }
            }
        }
    }
}.build()
