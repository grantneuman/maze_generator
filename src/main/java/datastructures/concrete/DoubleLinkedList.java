package datastructures.concrete;

import datastructures.interfaces.IList;
import misc.exceptions.EmptyContainerException;

import java.util.Iterator;
import java.util.NoSuchElementException;

/**
 * Replace this class with your DoubleLinkedList implementation from
 * project 1.
 */
public class DoubleLinkedList<T> implements IList<T> {

    private Node<T> front;
    private Node<T> back;
    private int size;

    public DoubleLinkedList() {
        this.front = null;
        this.back = null;
        this.size = 0;
    }

    @Override
    public void add(T item) {
        Node<T> newNode = new Node<T>(this.back, item, null);
        
        if (this.back != null) {
            this.back.next = newNode;
        }
        this.back = newNode;
        
        if (this.front == null) {
            this.front = newNode;
        }
        
        size++;
    }

    @Override
    public T remove() {
        if (this.size == 0) {
            throw new EmptyContainerException();
        }
        
        T deletedData = back.data;
        this.back = this.back.prev;
        if (this.size == 1) {
            this.front = null;
        } else {
            this.back.next = null;
        }
        
        this.size--;
        return deletedData;
    }
    
    @Override
    public T get(int index) {
        if (index < 0 || index >= this.size) {
            throw new IndexOutOfBoundsException();
        }
        
        Node<T> currNode = this.front;
        for (int i = 0; i < index; i++) {
            currNode = currNode.next;
        }
        
        return currNode.data;
    }
    
    @Override
    public void set(int index, T item) {
        if (index < 0 || index >= this.size) {
            throw new IndexOutOfBoundsException();
        }
        
        this.insert(index, item);
        this.delete(index + 1);
    }
    
    @Override
    public void insert(int index, T item) {
        if (index < 0 || index >= this.size + 1) {
            throw new IndexOutOfBoundsException();
        }
        
        Node<T> newNode = new Node<T>(null, item, null);
        if (size == 0) {
            this.front = newNode;
            this.back = newNode;
        } else {
            if (index == 0) {
                this.front.prev = newNode;
                newNode.next = this.front;
                this.front = newNode;
            } else if (index == this.size) {
                this.back.next = newNode;
                newNode.prev = this.back;
                this.back = newNode;
            } else  { 
                Node<T> currNode;

                // iterate from front or back depending on index
                if (this.size - index > this.size / 2) {
                    currNode = this.front;
                    for (int i = 0; i < index; i++) {
                        currNode = currNode.next;
                    }
                } else {
                    currNode = this.back;
                    for (int i = this.size - index - 1; i > 0; i--) {
                        currNode = currNode.prev;
                    }
                }        
                
                // adjust pointers and insert node
                newNode.next = currNode;
                newNode.prev = currNode.prev;
                currNode.prev.next = newNode;
                currNode.prev = newNode;
            }
        }
        
        size++;
     }

    @Override
    public T delete(int index) {
        if (index < 0 || index >= this.size) {
            throw new IndexOutOfBoundsException();
        }
        
        T deletedData;
        if (size == 1) {
            deletedData = this.back.data;
            this.front = null;
            this.back = null;
        } else {
            if (index == 0) {
                deletedData = this.front.data;
                this.front = this.front.next;
                this.front.prev = null;
            } else if (index == this.size - 1) {
                deletedData = this.back.data;
                this.back = this.back.prev;
                this.back.next = null;
            } else {
                Node<T> currNode;
                
                // iterate from front or back depending on index
                if (this.size - index > this.size / 2) {
                    currNode = this.front;
                    for (int i = 0; i < index; i++) {
                        currNode = currNode.next;
                    }
                } else {
                    currNode = this.back;
                    for (int i = this.size - index - 1; i > 0; i--) {
                        currNode = currNode.prev;
                    }
                }
                
                // adjust surrounding pointers and remove node
                deletedData = currNode.data;
                currNode.prev.next = currNode.next;
                currNode.next.prev = currNode.prev;            
            }
        }
        
        this.size--;
        return deletedData;
    }
    
    @Override
    public int indexOf(T item) {
        Node<T> currNode = this.front;
        for (int i = 0; i < this.size; i++) {
            if (objsEqual(currNode.data, item)) {
                return i;
            }
            currNode = currNode.next;
        }
        return -1;
    }
    
    @Override
    public int size() {
        return this.size;
    }
    
    public boolean contains(T other) {
        Node<T> currNode = this.front;
        for (int i = 0; i < this.size; i++) {
            if (objsEqual(currNode.data, other)) {
                return true;
            }
            currNode = currNode.next;
        }   
        
        return false;
    }
    
    /*
     * Returns true if given objects are equal, false if not
     */
    private boolean objsEqual(T data1, T data2) {
        if (data1 == data2) {
            return true;
        }
        if ((data1 == null) || (data2 == null))  {
            return false;
        }
        return data1.equals(data2);
    }
    
    @Override
    public Iterator<T> iterator() {
        return new DoubleLinkedListIterator<>(this.front);
    }

    private static class Node<E> {
        public final E data;
        public Node<E> prev;
        public Node<E> next;

        public Node(Node<E> prev, E data, Node<E> next) {
            this.data = data;
            this.prev = prev;
            this.next = next;
        }

        public Node(E data) {
            this(null, data, null);
        }
    }

    private static class DoubleLinkedListIterator<T> implements Iterator<T> {
        private Node<T> current;

        public DoubleLinkedListIterator(Node<T> current) {
            this.current = current;
        }

        /**
         * Returns 'true' if the iterator still has elements to look at;
         * returns 'false' otherwise.
         */
        public boolean hasNext() {
            return current != null;
        }

        /**
         * Returns the next item in the iteration and internally updates the
         * iterator to advance one element forward.
         *
         * @throws NoSuchElementException if we have reached the end of the iteration and
         *         there are no more elements to look at.
         */
        public T next() {
            if (!hasNext()) {
                throw new NoSuchElementException();
            }
            
            T result = current.data;
            current = current.next;
            return result;
        }
    }
}
