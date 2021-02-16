using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Text;
using System.Threading;

namespace BaseSLAM
{
    /// <summary>
    /// Concurrent queue with signaling feature
    /// </summary>
    /// <typeparam name="T">Type if item</typeparam>
    public class SignalConcurrentQueue<T> : ConcurrentQueue<T>
    {
        /// <summary>
        /// Signal which indicates enqueueing of the item
        /// </summary>
        public AutoResetEvent EnqueuedItemSignal { get; } = new AutoResetEvent(false);

        /// <summary>
        /// Signal which indicates dequeing of the item
        /// </summary>
        public AutoResetEvent DequeuedItemSignal { get; } = new AutoResetEvent(false);

        /// <summary>
        /// Enqueue item to the queue
        /// </summary>
        /// <param name="item"></param>
        public new void Enqueue(T item)
        {
            base.Enqueue(item);
            EnqueuedItemSignal.Set();
        }
        
        /// <summary>
        /// Try to dequeue item from the queue
        /// </summary>
        /// <param name="result"></param>
        /// <returns></returns>
        public new bool TryDequeue([MaybeNullWhen(false)] out T result)
        {
            if (base.TryDequeue(out result))
            {
                DequeuedItemSignal.Set();
                return true;
            }

            return false;
        }
    }
}
