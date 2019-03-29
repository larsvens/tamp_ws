#include "saasqp/rtisqp_wrapper.h"

// constructor
RtisqpWrapper::RtisqpWrapper()
{

    unsigned  i, j, iter;
    acado_timer t;
    real_t t1, t2;
    real_t fdbSum = 0.0;
    real_t prepSum = 0.0;
    int status;

    t1 = t2 = 0;

    // Reset all solver memory
    memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
    memset(&acadoVariables, 0, sizeof( acadoVariables ));

    // Initialize the solver
    acado_initializeSolver();

    // Prepare a consistent initial guess
    for (i = 0; i < N + 1; ++i)
    {
        acadoVariables.x[i * NX + 0] = 1;
        acadoVariables.x[i * NX + 1] = sqrt(1.0 - 0.1 * 0.1);
        acadoVariables.x[i * NX + 2] = 0.9;
        acadoVariables.x[i * NX + 3] = 0;
        acadoVariables.x[i * NX + 4] = 0;
        acadoVariables.x[i * NX + 5] = 0;
    }

    //
    // Prepare references
    //

    for (i = 0; i < N; ++i)
    {
        acadoVariables.y[i * NY + 0] = 0; // x
        acadoVariables.y[i * NY + 1] = 1.0; // y
        acadoVariables.y[i * NY + 2] = 0; // w
        acadoVariables.y[i * NY + 3] = 0;
        acadoVariables.y[i * NY + 4] = 0;
        acadoVariables.y[i * NY + 5] = 0;
        acadoVariables.y[i * NY + 6] = 0; // u
    }

    acadoVariables.yN[ 0 ] = 0; // x
    acadoVariables.yN[ 1 ] = 1.0; // y
    acadoVariables.yN[ 2 ] = 0; // w
    acadoVariables.yN[ 3 ] = 0;
    acadoVariables.yN[ 4 ] = 0;
    acadoVariables.yN[ 5 ] = 0;

    //
    // Current state feedback
    //
    for (i = 0; i < NX; ++i)
        acadoVariables.x0[ i ] = acadoVariables.x[ i ];

    //
    // Logger initialization
    //
    std::vector< std::vector< double > > log;

    log.resize(NUM_STEPS);
    for (i = 0; i < log.size(); ++i)
        log[ i ].resize(NX + NXA + NU + 5, 0.0);


    //
    // Warm-up the solver
    //
    acado_preparationStep();

    //
    // Real-time iterations loop
    //
    for( iter = 0; iter < NUM_STEPS; iter++ )
    {
        acado_tic( &t );
        status = acado_feedbackStep( );
        t2 = acado_toc( &t );

#if VERBOSE
        acado_printDifferentialVariables();
        acado_printControlVariables();
#endif // VERBOSE

        if ( status )
        {
            cout << "Iteration:" << iter << ", QP problem! QP status: " << status << endl;

            break;
        }

        //
        // Logging
        //

        for(i = 0; i < NX; i++)
            log[ iter ][ i ] = acadoVariables.x[ i ];
        for(j = 0;  i < NX + NXA + NU; i++, j++)
            log[ iter ][ i ] = acadoVariables.u[ j ];

        log[ iter ][ i++ ] = t1;
        log[ iter ][ i++ ] = t2;
        log[ iter ][ i++ ] = acado_getObjective();
        log[ iter ][ i++ ] = acado_getKKT();
        log[ iter ][ i++ ] = acado_getNWSR();

#if VERBOSE
        cout	<< "Iteration #" << setw( 4 ) << iter
                << ", KKT value: " << scientific << acado_getKKT()
                << ", objective value: " << scientific << acado_getObjective()
                << endl;
#endif // VERBOSE

        //
        // Prepare for the next iteration
        //

        // In this simple example, we feed the NMPC with an ideal feedback signal
        // i.e. what NMPC really expects in the next sampling interval
        for (i = 0; i < NX; ++i)
            acadoVariables.x0[ i ] = acadoVariables.x[NX + i];

        // Shift states and control and prepare for the next iteration
        acado_shiftStates(2, 0, 0);
        acado_shiftControls( 0 );


        acado_preparationStep();

    }

#if VERBOSE
    cout << "Average feedback time:    " << scientific << fdbSum / NUM_STEPS * 1e6 << " microseconds" << endl;
    cout << "Average preparation time: " << scientific << prepSum / NUM_STEPS * 1e6 << " microseconds" << endl;
#endif // VERBOSE


}

bool RtisqpWrapper::setInitialGuess(common::Trajectory traj){
    for (uint k = 0; k < N + 1; ++k)
    {
        acadoVariables.x[k * NX + 0] = double(traj.s.at(k)); // s
        acadoVariables.x[k * NX + 1] = double(traj.d.at(k)); // d
        acadoVariables.x[k * NX + 2] = double(traj.deltapsi.at(k)); // deltapsi
        acadoVariables.x[k * NX + 3] = double(traj.psidot.at(k)); // psidot
        acadoVariables.x[k * NX + 4] = double(traj.vx.at(k)); // vx
        acadoVariables.x[k * NX + 5] = double(traj.vy.at(k)); // vy
        acadoVariables.x[k * NX + 6] = double(traj.kappac.at(k)); // kappac
    }
    return true;
}

bool RtisqpWrapper::setReference(common::Trajectory traj){
    for (uint k = 0; k < N; ++k)
    {
        acadoVariables.y[k * NY + 0] = double(traj.s.at(k)); // s
        acadoVariables.y[k * NY + 1] = double(traj.d.at(k)); // d
        acadoVariables.y[k * NY + 2] = double(traj.deltapsi.at(k)); // deltapsi
        acadoVariables.y[k * NY + 3] = double(traj.psidot.at(k)); // psidot
        acadoVariables.y[k * NY + 4] = double(traj.vx.at(k)); // vx
        acadoVariables.y[k * NY + 5] = double(traj.vy.at(k)); // vy
        acadoVariables.y[k * NY + 6] = double(traj.kappac.at(k)); // kappac
        acadoVariables.y[k * NY + 7] = double(traj.Fyf.at(k)); // Fyf
        acadoVariables.y[k * NY + 8] = double(traj.Fx.at(k)); // Fx
    }

    acadoVariables.yN[ 0 ] = double(traj.s.at(N)); // s
    acadoVariables.yN[ 1 ] = double(traj.d.at(N)); // d
    acadoVariables.yN[ 2 ] = double(traj.deltapsi.at(N)); // deltapsi
    acadoVariables.yN[ 3 ] = double(traj.psidot.at(N)); // psidot
    acadoVariables.yN[ 4 ] = double(traj.vx.at(N)); // vx
    acadoVariables.yN[ 5 ] = double(traj.vy.at(N)); // vy
    // acadoVariables.yN[ 6 ] = double(traj.kappac.at(N)); // kappac

    return true;
}

bool RtisqpWrapper::setInitialState(common::State state){
    acadoVariables.x0[0] = double(state.s);
    acadoVariables.x0[1] = double(state.d);
    acadoVariables.x0[2] = double(state.deltapsi);
    acadoVariables.x0[3] = double(state.psidot);
    acadoVariables.x0[4] = double(state.vx);
    acadoVariables.x0[5] = double(state.vy);
    return true;
}

bool RtisqpWrapper::shiftStateAndControls(){
    acado_shiftStates(2, 0, 0);
    acado_shiftControls( 0 );
    return true;
}

bool RtisqpWrapper::doPreparationStep(){
    // todo: put timer
    acado_preparationStep();
    return true;
}

int RtisqpWrapper::doFeedbackStep(){
    // todo: put timer
    int status = acado_feedbackStep();
    return status;
}

common::Trajectory getTrajectory(){
    common::Trajectory traj;
    // todo
    return traj;
}

