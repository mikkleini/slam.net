using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace BaseSLAM
{
    /// <summary>
    /// Parallel worker.
    /// This is like .NET Parallel.For function, but it's faster.
    /// In this case the threads are already created and it doesn't come with limitations of the Parallel.For.
    /// </summary>
    public class ParallelWorker : IDisposable
    {
        private readonly Thread[] threads;
        private readonly EventWaitHandle[] startSignals;
        private readonly EventWaitHandle[] endSignals;
        private readonly SignalConcurrentQueue<WorkItem>[] actionQueues;
        private readonly CancellationTokenSource cts;

        private struct WorkItem
        {
            public Action<int> Action;
            public AutoResetEvent WaitHandle;
        }

        /// <summary>
        /// Consturctor
        /// </summary>
        /// <param name="numThreads">Number of threads to run in parallel</param>
        /// <param name="name">Name of the worker</param>
        public ParallelWorker(int numThreads, string name = "Worker")
        {
            NumThreads = numThreads;

            threads = new Thread[numThreads];
            startSignals = new EventWaitHandle[numThreads];
            endSignals = new EventWaitHandle[numThreads];
            actionQueues = new SignalConcurrentQueue<WorkItem>[numThreads];
            cts = new CancellationTokenSource();

            for (int i = 0; i < numThreads; i++)
            {
                threads[i] = new Thread(new ParameterizedThreadStart(WorkLoop))
                {
                    Name = $"{name} #{i + 1}"
                };

                startSignals[i] = new AutoResetEvent(false);
                endSignals[i] = new AutoResetEvent(false);
                actionQueues[i] = new SignalConcurrentQueue<WorkItem>();
                threads[i].Start(i);
            }
        }

        /// <summary>
        /// Number of parallel threads
        /// </summary>
        public int NumThreads { get; private set; }

        /// <summary>
        /// Thread work loop
        /// </summary>
        /// <param name="data"></param>
        private void WorkLoop(object data)
        {
            int index = (int)data;
            WaitHandle[] waitHandles = new WaitHandle[] { cts.Token.WaitHandle, actionQueues[index].EnqueuedItemSignal };

            while (!cts.IsCancellationRequested)
            {
                switch (WaitHandle.WaitAny(waitHandles))
                {
                    case 0:
                        // Cancellation
                        break;

                    case 1:

                        if (actionQueues[index].TryDequeue(out WorkItem item))
                        {
                            item.Action(index);
                            item.WaitHandle?.Set();
                        }
                        
                        break;
                }
            }
        }

        /// <summary>
        /// Work with the function in parallel threads.
        /// Blocking function, non-reentrant
        /// </summary>
        /// <param name="function">Function to run from threads</param>
        public void Work(Action<int> function, bool waitResult = true)
        {
            AutoResetEvent[] waitHandles = new AutoResetEvent[NumThreads];

            for (int i = 0; i < NumThreads; i++)
            {
                waitHandles[i] = waitResult ? new AutoResetEvent(false) : null;

                actionQueues[i].Enqueue(new WorkItem()
                {
                    Action = function,
                    WaitHandle = waitHandles[i]
                });
            }

            if (waitResult)
            {
                WaitHandle.WaitAll(waitHandles);
            }
        }

        /// <summary>
        /// Dispose function
        /// </summary>
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Internal disposing function.
        /// </summary>
        /// <param name="disposing"></param>
        protected virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                cts.Cancel();
                threads.ForEach(t => t.Join());
            }
        }
    }
}
